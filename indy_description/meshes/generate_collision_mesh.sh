#!/bin/bash

# =========================================================
# Usage:
# ./generate_collision_mesh.sh -i visual_mesh.stl -h 3 -l 4
# =========================================================

# Set the full path to the TestVHACD executable
testvhacd_executable="${HOME}/git/v-hacd/app/build/TestVHACD"

# See TestVHACD documentation for more information about the parameters
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# Default value of the hull parameter
#   ***>> THIS PARAMETER IS IMPORTANT <<***
hull_param=0

# Default value of the minimum size of a voxel edge
# Good value for the minimum edge size is different for each mesh
# Try the values between 1 ~ 30. Too small or too large values may lose details.
#   ***>> THIS PARAMETER IS IMPORTANT <<***
minedge_param=15

# Default value of the voxel resolution
voxel_param=100000

# Default value of the maximum number of vertices
max_vertices=128

# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

# Parse the command-line arguments
while getopts "i:h:l:r:v" opt; do
  case $opt in
    i)
      input_file="$OPTARG"
      ;;
    h)
      hull_param="$OPTARG"
      ;;
    l)
      minedge_param="$OPTARG"
      ;;
    r)
      voxel_param="$OPTARG"
      ;;
    v)
      max_vertices="$OPTARG"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

# Check if MeshLab is installed
if ! [ -x "$(command -v meshlabserver)" ]; then
  echo "Error: MeshLab is not installed." >&2
  exit 1
fi

# Get the input file name without extension
input_filename=$(basename "${input_file%.*}")

# Convert STL to Wavefront OBJ
meshlabserver -i "$input_file" -o "temp_${input_filename}.obj"

# Approximate convex decomposition
"${testvhacd_executable}" "temp_${input_filename}.obj" -h "$hull_param" -l "$minedge_param" -r "$voxel_param" -v "$max_vertices" -e 0.001

# Rename the output file
mv decomp.obj "${input_filename}.obj"

# Remove unnecessary files
rm "temp_${input_filename}.obj" decomp.stl # decomp.mtl

echo "Collision mesh generated: ${input_filename}.obj"
