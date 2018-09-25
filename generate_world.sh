#!/bin/bash

content="<?xml version=\"1.0\" ?>
<sdf version=\"1.5\">
  <world name=\"default\">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <!-- Own physics settings to speed up simulation -->
    <physics type='ode'>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>"

obstacle_cnt=1
coordinate_file="$1"
while IFS=';' read -ra line
do
    content+="<model name=\"obstacle$obstacle_cnt\">
      <static>true</static>
      <link name=\"link\">
        <collision name=\"collision$obstacle_cnt\">"
    geom_content="<geometry>
                    <polyline>"

    for coordinate in "${line[@]}"; do
    	geom_content+="<point>$coordinate</point>"
    done
    geom_content+="<height>1</height>
            </polyline>
          </geometry>"
    content+=$geom_content
    content+="</collision>
        <visual name=\"visual$obstacle_cnt\">"
    content+=$geom_content
    content+="<material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>  
      </link>
    </model>"
    let obstacle_cnt+=1
done < "$coordinate_file"

content+="</world>
         </sdf>"
echo $content
