"""Joint drive gains for a position-controlled arm, derived from its URDF.

Isaac Sim (PhysX) drives each joint with an implicit PD law:

    torque = stiffness * (q_target - q) + damping * (qd_target - qd)

I don't want to guess stiffness and damping, so I derive them per joint from
two requirements, using the masses and inertias in the URDF:

1. Response: the joint behaves like a second-order system with natural
   frequency omega_n and damping ratio zeta, given the inertia it moves:
       stiffness = I * omega_n**2
2. Holding against gravity: with a pure PD drive (no gravity compensation)
   the joint sags until stiffness * error = gravity torque. To keep that sag
   below max_static_error:
       stiffness >= tau_gravity_max / max_static_error

I take the larger stiffness and then set damping for the requested zeta:
       damping = 2 * zeta * sqrt(stiffness * I)

I is the inertia of everything downstream of the joint about its axis, and
tau_gravity_max is the worst case over the joint's own rotation, both with the
other joints at zero. That is a reference pose, not a bound over the whole
workspace, which is why the report also gives the margin to the goal
tolerance.

Pure Python + NumPy, no Isaac Sim imports, so it runs and is tested anywhere:

    python isaac/drive_gains.py isaac/build/abb_irb1200.urdf
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])


@dataclass
class Inertial:
    mass: float
    com: np.ndarray  # in the link frame
    inertia: np.ndarray  # 3x3 about the COM, in the inertial frame
    rotation: np.ndarray  # inertial frame -> link frame


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray  # 4x4, parent link frame -> joint frame
    axis: np.ndarray  # in the joint frame


@dataclass
class JointGains:
    joint: str
    inertia: float  # kg m^2 about the axis
    gravity_torque: float  # N m, worst case over the joint's rotation
    stiffness: float  # N m / rad
    damping: float  # N m s / rad
    natural_frequency: float  # rad/s with the chosen stiffness
    static_error: float  # rad, sag under the worst-case gravity torque
    limited_by: str  # "response" or "gravity"


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is not None:
        xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
        transform[:3, :3] = rpy_to_matrix(*rpy)
        transform[:3, 3] = xyz
    return transform


def parse_urdf(text: str) -> tuple[str, dict[str, Inertial | None], dict[str, Joint]]:
    """Return the root link, the inertial data per link and the joints by child link."""
    robot = ET.fromstring(text)
    links: dict[str, Inertial | None] = {}
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            links[link.get("name")] = None
            continue
        origin = _origin(inertial.find("origin"))
        i = inertial.find("inertia").attrib
        ixx, iyy, izz = float(i["ixx"]), float(i["iyy"]), float(i["izz"])
        ixy, ixz, iyz = (float(i.get(k, 0.0)) for k in ("ixy", "ixz", "iyz"))
        links[link.get("name")] = Inertial(
            mass=float(inertial.find("mass").get("value")),
            com=origin[:3, 3],
            inertia=np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]),
            rotation=origin[:3, :3],
        )

    joints: dict[str, Joint] = {}
    for joint in robot.findall("joint"):
        axis = joint.find("axis")
        joints[joint.find("child").get("link")] = Joint(
            name=joint.get("name"),
            type=joint.get("type"),
            parent=joint.find("parent").get("link"),
            child=joint.find("child").get("link"),
            origin=_origin(joint.find("origin")),
            axis=np.array([float(v) for v in (axis.get("xyz") if axis is not None else "1 0 0").split()]),
        )
    roots = [name for name in links if name not in joints]
    if len(roots) != 1:
        raise ValueError(f"Expected one root link, found {roots}")
    return roots[0], links, joints


def link_poses(root: str, joints: dict[str, Joint]) -> dict[str, np.ndarray]:
    """World pose of every link with all joints at zero."""
    poses = {root: np.eye(4)}
    pending = dict(joints)
    while pending:
        progressed = False
        for child, joint in list(pending.items()):
            if joint.parent in poses:
                poses[child] = poses[joint.parent] @ joint.origin
                del pending[child]
                progressed = True
        if not progressed:
            raise ValueError(f"Disconnected joints: {sorted(pending)}")
    return poses


def descendants(link: str, joints: dict[str, Joint]) -> list[str]:
    children = [j.child for j in joints.values() if j.parent == link]
    result = list(children)
    for child in children:
        result += descendants(child, joints)
    return result


def axis_properties(joint: Joint, links, joints, poses) -> tuple[float, float]:
    """Inertia about the joint axis and worst-case gravity torque, at zero pose."""
    frame = poses[joint.child]  # the joint frame coincides with the child link frame
    axis = frame[:3, :3] @ (joint.axis / np.linalg.norm(joint.axis))
    point = frame[:3, 3]

    inertia = 0.0
    first_moment = np.zeros(3)  # sum of m * (COM offset perpendicular to the axis)
    for name in [joint.child] + descendants(joint.child, joints):
        body = links[name]
        if body is None or body.mass == 0.0:
            continue
        pose = poses[name]
        com = pose[:3, :3] @ body.com + pose[:3, 3]
        rotation = pose[:3, :3] @ body.rotation
        offset = com - point
        perpendicular = offset - axis * (offset @ axis)
        inertia += axis @ (rotation @ body.inertia @ rotation.T) @ axis
        inertia += body.mass * (perpendicular @ perpendicular)
        first_moment += body.mass * perpendicular

    # Rotating the subtree about the axis, the gravity torque is at most
    # |sum m r_perp| times the part of gravity perpendicular to the axis.
    gravity_perpendicular = GRAVITY - axis * (GRAVITY @ axis)
    gravity_torque = np.linalg.norm(first_moment) * np.linalg.norm(gravity_perpendicular)
    return float(inertia), float(gravity_torque)


def compute_gains(
    urdf_text: str,
    natural_frequency: float = 2 * math.pi * 5.0,
    damping_ratio: float = 1.0,
    max_static_error: float = 0.002,
) -> list[JointGains]:
    root, links, joints = parse_urdf(urdf_text)
    poses = link_poses(root, joints)
    gains = []
    for joint in joints.values():
        if joint.type not in ("revolute", "continuous"):
            continue
        inertia, gravity_torque = axis_properties(joint, links, joints, poses)
        k_response = inertia * natural_frequency**2
        k_gravity = gravity_torque / max_static_error
        stiffness = max(k_response, k_gravity)
        gains.append(JointGains(
            joint=joint.name,
            inertia=inertia,
            gravity_torque=gravity_torque,
            stiffness=stiffness,
            damping=2 * damping_ratio * math.sqrt(stiffness * inertia),
            natural_frequency=math.sqrt(stiffness / inertia),
            static_error=gravity_torque / stiffness,
            limited_by="gravity" if k_gravity > k_response else "response",
        ))
    return sorted(gains, key=lambda g: g.joint)


def report(gains: list[JointGains], dt: float) -> str:
    lines = [
        f"{'joint':8} {'I kg m2':>9} {'tau_g Nm':>9} {'k Nm/rad':>10} {'c Nms/rad':>10} "
        f"{'wn rad/s':>9} {'wn*dt':>6} {'sag rad':>8}  limited by",
    ]
    for g in gains:
        lines.append(
            f"{g.joint:8} {g.inertia:9.4f} {g.gravity_torque:9.2f} {g.stiffness:10.1f} "
            f"{g.damping:10.2f} {g.natural_frequency:9.1f} {g.natural_frequency * dt:6.2f} "
            f"{g.static_error:8.4f}  {g.limited_by}"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("urdf")
    parser.add_argument("--frequency-hz", type=float, default=5.0,
                        help="Natural frequency of each joint's response.")
    parser.add_argument("--damping-ratio", type=float, default=1.0)
    parser.add_argument("--max-static-error", type=float, default=0.002,
                        help="Largest gravity sag allowed, rad.")
    parser.add_argument("--dt", type=float, default=1 / 60, help="Physics step, s.")
    args = parser.parse_args()
    with open(args.urdf) as f:
        gains = compute_gains(f.read(), 2 * math.pi * args.frequency_hz,
                              args.damping_ratio, args.max_static_error)
    print(report(gains, args.dt))


if __name__ == "__main__":
    main()
