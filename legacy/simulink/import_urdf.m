% Import the original URDF from any working directory (requires Robotics System Toolbox).
folder=fileparts(mfilename('fullpath'));
root=fileparts(fileparts(folder));
robot=importrobot(fullfile(root,'simulink','UR16e.urdf'),'MeshPath',fullfile(root,'simulink','model'));
robot.DataFormat='column';robot.Gravity=[0 0 -9.81];
show(robot);
