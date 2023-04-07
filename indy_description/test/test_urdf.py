# Copyright 2023 RISE LAB, Neuromeka Co.,Ltd.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the RISE LAB, Neuromeka Co.,Ltd. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from os import path, scandir
from ament_index_python.packages import get_package_share_directory
import yaml
import xacro
import xml

robots_folder = path.join(get_package_share_directory("indy_description"), "robots")
mesh_folder = path.join(get_package_share_directory("indy_description"), "meshes")


def get_model_list():
    # Read folder names under `meshes` folder
    model_list = [f.name for f in scandir(mesh_folder) if f.is_dir()]
    # model_list = ["indy7", "indy7_v2", "indy12", "indyrp2", "indyrp2_v2"]
    return model_list


def test_file_exists():
    # Check if the Xacro file exists
    xacro_file = path.join(robots_folder, "indy_arm.urdf.xacro")
    assert path.exists(xacro_file), f"Xacro file not found: {xacro_file}"

    config_list = ["joint_limits.yaml", "kinematics.yaml", "physical_parameters.yaml"]
    model_list = get_model_list()

    for model in model_list:
        # Check if the config files exist
        for cfg in config_list:
            config_file = path.join(robots_folder, model, cfg)
            assert path.exists(config_file), f"Config file not found: {config_file}"

        # Read the number of joints from the YAML, `kinematics.yaml`
        with open(path.join(robots_folder, model, "kinematics.yaml"), "r") as f:
            kinematics_file = yaml.safe_load(f)
            num_joints = len(kinematics_file["kinematics"])

        # Check if the mesh files exist
        for i in range(num_joints):
            # Visual mesh
            mesh_file = path.join(mesh_folder, model, "visual", f"{model}_{i}.stl")
            assert path.exists(mesh_file), f"Visual mesh file not found: {mesh_file}"
            # Collision mesh
            mesh_file = path.join(mesh_folder, model, "collision", f"{model}_{i}.obj")
            assert path.exists(mesh_file), f"Collision mesh file not found: {mesh_file}"


def test_xacro_parsing():
    # Check if the Xacro file can be converted to URDF
    xacro_file = path.join(robots_folder, "indy_arm.urdf.xacro")
    model_list = get_model_list()

    for model in model_list:
        try:
            xacro.process_file(xacro_file, mappings={"model": model, "use_fake_hardware": "False"})
        except xml.parsers.expat.ExpatError as e:
            assert False, f"Xacro parsing failed. Model: {model}, Error: {e}"


if __name__ == "__main__":
    import pytest

    pytest.main([__file__])
