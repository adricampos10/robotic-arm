#!/bin/bash
# Export a plain URDF of the arm, plus the mesh packages it references, from
# the ROS 2 image into isaac/build/, ready for Isaac Sim's URDF importer.
#
#   isaac/export_urdf.sh                # visual meshes as upstream (.dae)
#   isaac/export_urdf.sh --stl-visuals  # use the collision .stl files as visuals too
#
# Needs the image: docker build -f docker/Dockerfile -t robotic-arm:jazzy .
set -euo pipefail

cd "$(dirname "$0")/.."
OUT=isaac/build
mkdir -p "$OUT/packages"

docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD/$OUT:/out" robotic-arm:jazzy bash -c '
  set -e
  xacro "$(ros2 pkg prefix abb_cell_description)/share/abb_cell_description/urdf/abb_cell.urdf.xacro" \
    hardware:=isaac -o /out/abb_irb1200.urdf
  for pkg in abb_irb1200_support abb_resources; do
    rm -rf "/out/packages/$pkg"
    cp -r "$(ros2 pkg prefix $pkg)/share/$pkg" "/out/packages/$pkg"
  done
'

if [[ "${1:-}" == "--stl-visuals" ]]; then
  # The upstream visual and collision meshes share file names.
  sed -i.bak -E 's#meshes/irb1200_5_90/visual/([a-z_0-9]+)\.dae#meshes/irb1200_5_90/collision/\1.stl#' \
    "$OUT/abb_irb1200.urdf" && rm "$OUT/abb_irb1200.urdf.bak"
fi

echo "Wrote $OUT/abb_irb1200.urdf and $OUT/packages/"
