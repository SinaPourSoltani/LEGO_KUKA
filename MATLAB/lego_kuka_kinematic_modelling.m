% Requires:
    % Robotics System Toolbox

clear;
clc;
close all;

% Add the DH parameters based on your robot model:
% a, alpha, d, theta
%𝒂𝒊 – The distance from 𝒁𝑖 to 𝒁𝑖+1 measured along 𝑿𝑖+1 
%𝜶𝒊 – The angle from 𝒁𝑖 to 𝒁𝑖+1 measured about 𝑿𝑖+1
%𝒅𝒊 – The distance from 𝑿𝑖 to 𝑿𝑖+1 measured along 𝒁𝑖 
%𝜽𝒊 – The angle from 𝑿𝑖 to 𝑿𝑖 measured about 𝒁𝑖
dhparams = [0   	0       0.0438	0; 	
            0.0159  -pi/2   0.0877  0;
            0.1116 	pi      0	    0;
            0.0399 	pi/2    0	    0;
            0    	0     	0.0956 	0;
            0	    pi/2    0.0638  0;
            0       -pi/2   0       0;
            0   	0       0.0478  0;
            0   	0       0       0;];

% Measurements of the LEGO KUKA Robot [m]
robot_dims = {0.135, 0.135, 0.115; % box: l w h
                0.040, 0.160, 0.0; % cyl: r h
                0.025, 0.185, 0.0;
                0.001, 0.020, 0.0;
                0.0375, 0.070, 0.0;
                0.018, 0.045, 0.0;
                0.001, 0.020, 0.0;
                0.0325, 0.025 0.0;
                0.015, 0.005 0.0;};

% Define the robot as a rigid body object:
robot = rigidBodyTree;

% Initialise empty links and joint array
links = cell(8,1);
joints = cell(8,1);

% Iterate  through the  DH Parameters matrix to set 
% transforms between each joint. Basically creating Parent
% and children relation for the frames. 
for i = 1:9
    type = "revolute";
    if i == 4 || i == 7
        type = "fixed";
    end
    links{i} = rigidBody(['link' num2str(i)]);
    joints{i} = rigidBodyJoint(['joint' num2str(i)], type);
    coll = NaN;

    if i == 1
        coll = collisionBox(robot_dims{i,:});
    else
        coll = collisionCylinder(robot_dims{i,1:2});
    end
    %coll.Pose = DH2trans(dhparams(i,:));
    %addCollision(links{i}, coll);    
    
    % Apply DH parameters to describe the relative transform for the joints in space
    setFixedTransform(joints{i}, dhparams(i,:), "dh");
    % Apply the specific joint to the link
    links{i}.Joint = joints{i};
    
    if i == 1
        addBody(robot, links{i}, "base");
    else
        addBody(robot, links{i}, links{i-1}.Name);
    end
end

config = homeConfiguration(robot);
config(3).JointPosition = -pi/2;
config(6).JointPosition = pi/2;
config(7).JointPosition = -pi/2;

showdetails(robot)

figure(Name="KUKA Robot Model");
show(robot, config);

%gui = interactiveRigidBodyTree(robot, config, MarkerScaleFactor=0.5);