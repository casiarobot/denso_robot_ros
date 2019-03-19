clc;clear;
addpath(genpath('/data/denso_ws/src/denso_robot_ros/calibration_flow/scripts/5_solveXZ/matlab/yamlMatlab')); 
A_PATH = '/data/denso_ws/src/denso_robot_ros/calibration_flow/scripts/4_cameraPoseEstimation/goal/As.yaml';
B_PATH = '/data/denso_ws/src/denso_robot_ros/calibration_flow/scripts/3_autoPose/goal/Bs.yaml';
As = ReadYaml(A_PATH);
Bs = ReadYaml(B_PATH);



x= 0
e = obj(x, As, Bs)