% main_simulation.m
clear;
close all;
clc;

% Add youBot directory to the MATLAB path
addpath("Pioneer\");
addpath("functions\");

% Initialize parameters and objects
[robot, K, cRp, ctp, sd,tagSize, apriltagCornerPoints, lamda, simTime] = initialize();       

robot.Simulation_Step();

% Start simulation timer
tic;

% Data arrays for plotting
[time_plot, con_plot, v_plot, actual_plot, desired_plot] = initializeDataArrays();

timeStep = 0.05; % Adjust based on your desired simulation frequency (20 Hz)



% Initialize figure and plot handles
figure(1);
hImage = imshow(zeros(size(robot.Get_Image()))); hold on;

% Initialize plot handles with NaN data
hDetectedPoints = plot(NaN, NaN, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hDesiredPoints = plot(NaN, NaN, 'gx', 'MarkerSize', 10, 'LineWidth', 2);

legend('Detected Corners', 'Desired Corners');
title('AprilTag Corner Points Matching');
hold off;
vr_log = [];

while toc <= simTime
    % Start timer for the loop
    loopStartTime = tic;      

    image = robot.Get_Image(); % Getting image from robot

    % Update the image data
    set(hImage, 'CData', image);

    % Detect April tag and compute control inputs if tag detected
    [id, uv, Rt_pose] = detectAprilTag(image, K, tagSize);
    if ~isempty(id)
        % Process tag detection and compute velocities
        [vp, vr, s] = computeVelocities(uv, Rt_pose, K, apriltagCornerPoints, cRp, ctp, lamda, sd);
        robot.Set_Joint_Velocity(vr); 
        vr_log = [vr_log vr];

        % Update detected corner points
        set(hDetectedPoints, 'XData', uv(:, 1), 'YData', uv(:, 2));

        % Compute desired points in image coordinates
        sd_image = computeDesiredImagePoints(sd, K);

        % Update desired corner points
        set(hDesiredPoints, 'XData', sd_image(:, 1), 'YData', sd_image(:, 2));
    else
        % If no tag is detected, clear the detected points
        set(hDetectedPoints, 'XData', NaN, 'YData', NaN);
    end
    drawnow;
    robot.Simulation_Step();

    % Wait to maintain real-time step duration
    while toc(loopStartTime) < timeStep
        pause(0.001); % Small pause to prevent busy-waiting
    end
end


% Plot results
% plotResults(time_plot, actual_plot, desired_plot, con_plot, v_plot);

function sd_image = computeDesiredImagePoints(sd, K)
    % Reshape sd to get s1, s2, s3, s4
    s_desired = reshape(sd, 2, [])';  % Each row is [x, y]

    % Get the numeric intrinsic matrix
    intrinsicMatrix = K.IntrinsicMatrix';  % Transpose to match expected format

    % Convert normalized image coordinates to pixel coordinates
    numPoints = size(s_desired, 1);
    sd_image = zeros(numPoints, 2);
    for i = 1:numPoints
        s_norm = s_desired(i, :)';
        uv_homogeneous = intrinsicMatrix * [s_norm; 1];
        sd_image(i, :) = uv_homogeneous(1:2) ./ uv_homogeneous(3);
    end
end
