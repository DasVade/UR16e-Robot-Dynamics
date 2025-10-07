clc;
clear;

addpath(genpath('.'));

robot = importrobot("UR16e.urdf");
show(robot)

rmpath(genpath('.'))