% Import the original URDF from any working directory (requires Robotics System Toolbox).
folder=fileparts(mfilename('fullpath'));
robot=importrobot(fullfile(folder,'UR16e.urdf'),'MeshPath',fullfile(folder,'model'));
robot.DataFormat='column';robot.Gravity=[0 0 -9.81];
show(robot);
