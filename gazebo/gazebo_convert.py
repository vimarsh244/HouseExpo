import numpy as np
import matplotlib as plt
import os
import random

import json

def convert_json_to_gazebo(json_data):
    # data = json.loads(json_data)
    data = json_data
    verts = data['verts']
    bbox_min = data['bbox']['min']
    bbox_max = data['bbox']['max']

    def create_wall_segment(v1, v2, wall_id):
        return f"""
        <model name='wall_{wall_id}'>
            <static>true</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>{abs(v2[0] - v1[0])} {abs(v2[1] - v1[1])} 1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>{abs(v2[0] - v1[0])} {abs(v2[1] - v1[1])} 1</size>
                            
                        </box>
                    </geometry>
                </visual>
            </link>
            <pose>{(v1[0] + v2[0]) / 2} {(v1[1] + v2[1]) / 2} 0.5 0 0 0</pose>
        </model>
        """

    def create_boundary_wall(min_pt, max_pt):
        return f"""
        <model name='boundary_wall_1'>
            <static>true</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>{abs(max_pt[0] - min_pt[0])} 0.1 1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>{abs(max_pt[0] - min_pt[0])} 0.1 1</size>
                        </box>
                    </geometry>
                </visual>
            </link>
            <pose>{(min_pt[0] + max_pt[0]) / 2} {min_pt[1]} 0.5 0 0 0</pose>
        </model>
        <model name='boundary_wall_2'>
            <static>true</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>{abs(max_pt[0] - min_pt[0])} 0.1 1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>{abs(max_pt[0] - min_pt[0])} 0.1 1</size>
                        </box>
                    </geometry>
                </visual>
            </link>
            <pose>{(min_pt[0] + max_pt[0]) / 2} {max_pt[1]} 0.5 0 0 0</pose>
        </model>
        <model name='boundary_wall_3'>
            <static>true</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>0.1 {abs(max_pt[1] - min_pt[1])} 1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>0.1 {abs(max_pt[1] - min_pt[1])} 1</size>
                        </box>
                    </geometry>
                </visual>
            </link>
            <pose>{min_pt[0]} {(min_pt[1] + max_pt[1]) / 2} 0.5 0 0 0</pose>
        </model>
        <model name='boundary_wall_4'>
            <static>true</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>0.1 {abs(max_pt[1] - min_pt[1])} 1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>0.1 {abs(max_pt[1] - min_pt[1])} 1</size>
                        </box>
                    </geometry>
                </visual>
            </link>
            <pose>{max_pt[0]} {(min_pt[1] + max_pt[1]) / 2} 0.5 0 0 0</pose>
        </model>
        """

    wall_elements = ""
    for i in range(len(verts) - 1):
        wall_elements += create_wall_segment(verts[i], verts[i + 1], i)

    boundary_wall = create_boundary_wall(bbox_min, bbox_max)

    sun_light = """
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    """
    
    other_imports = """
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    """

    gazebo_world = f"""
    <?xml version="1.0" ?>
    <sdf version="1.6">
        <world name="default">
            {sun_light}
            {other_imports}
            {wall_elements}
            {boundary_wall}
        </world>
    </sdf>
    """

    return gazebo_world

# select one json file randomly from the json folder

def select_random_json_file(folder_path):
    files = os.listdir(folder_path)
    json_files = [file for file in files if file.endswith('.json')]
    if not json_files:
        raise FileNotFoundError("No JSON files found in the specified folder.")
    random_file = random.choice(json_files)
    return os.path.join(folder_path, random_file)

folder_path = "../HouseExpo/json"

try:
    random_json_file = select_random_json_file(folder_path)
    print(f"Selected JSON file: {random_json_file}")
except FileNotFoundError as e:
    print(e)

json_content  =json.load(open(random_json_file))

world_file = convert_json_to_gazebo(json_content)

with open("model.world", "w") as f:
    f.write(world_file)

# run gazebo
os.system("gazebo model.world --verbose")
