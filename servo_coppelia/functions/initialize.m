% initialize.m
function [bot, K, cRp, ctp, sd, tagSize, apriltagCornerPoints, lamda, simTime] = initialize()   
    % Transformation from camera to platform
    cRp = [0 -1 0; 
           0 0 -1;
           1 0 0];
    t = [0.25 0 0]; % Camera position from the platform
    ctp = [0 -t(3) t(2); 
           t(3) 0 -t(1); 
           -t(2) t(1) 0];

    % Desired image features (normalized image coordinates)
    sd = [-0.0844; 0.1060; 0.1129; 0.1060; 0.1127; -0.0906; -0.0844; -0.0908];

    % Control gain
    lamda = 10;
    
    % Initialize youBot Platform
    bot = Bot_Pioneer();

    % Simulation Time
    simTime = 120;

    % Camera intrinsic parameters
    focalLength = [588.053791, 587.981269];
    principalPoint = [319.476121, 239.606921];
    imageSize = [480, 640];
    K = cameraIntrinsics(focalLength, principalPoint, imageSize);

    % Apriltag parameters
    tagSize = 0.3; % size (meters)
    apriltagCornerPoints = [-tagSize/2 tagSize/2 0 1;
                             tagSize/2 tagSize/2 0 1;
                             tagSize/2 -tagSize/2 0 1;
                            -tagSize/2 -tagSize/2 0 1]';
end


