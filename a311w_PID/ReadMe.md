<?xml version="1.0"?>
<!--
	EE3305/ME3243
	Name: Chen Guoyi (guoyi@comp.nus.edu.sg)
	Matric Number: A0262311W
-->
<launch>
	<arg name="model" default="burger"/>
	<arg name="x" default="0.5"/>
	<arg name="y" default="0.5"/>
	<arg name="yaw" default="1.57"/>
	<!-- Launch Gazebo. -->
	<include file="$(find gazebo_ros)/launch/empty_world.launch">
	<arg name="world_name" value="$(find a345b_path)/worlds/EE3305_2022.world"/>
	<arg name="paused" value="false"/>
	<arg name="use_sim_time" value="true"/>
	<arg name="gui" value="true"/>
	<arg name="headless" value="false"/>
	<arg name="debug" value="false"/>
	</include
	<!-- Use Gazebo service to spawn the robot. -->
	<param name="robot_description" command="$(find xacro)/xacro --inorder $(find
	turtlebot3_description)/urdf/turtlebot3 $(arg model).urdf.xacro"/>
	<node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model
	turtlebot3_$(arg model) -x $(arg x) -y $(arg y) -Y $(arg yaw) -param
	robot_description"/>
</launch>