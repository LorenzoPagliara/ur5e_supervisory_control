% --------------------------------------------------------------------
%
%  This module has been developed by the Automatic Control Group
%  of the University of Salerno, Italy.
%
%  Title:    plot_joint_state.m
%  Author:   Vincenzo Petrone
%  Org.:     UNISA
%  Date:     Feb 2, 2025
%
%  Reads a bag file containing a sequence of joint state messages from
%  two topic, containing ground truth (GT) and computed torques; the
%  former are measured from the robot, while the latter are estimated
%  by the inverse dynamics solver. The read data are displayed in
%  static and animated plots.
%
% --------------------------------------------------------------------

%% Setup

% Init
clc;
clear;
close all;

% Plot configuration
FS = 20;  % FontSize
LW = 3;  % LineWidth

% Define file paths
bag_files = [
    ""
    ""
];
topic = '/torques';
output_dir = "";

%% Open bag files and plot torques

for ii = 1 : length(bag_files)
    % Get bag file
    bag_file = bag_files(ii);
    disp("Opening " + bag_file + "...");

    % Open input bag
    input_bag = ros2bagreader(bag_file);
    input_msgs = readMessages(select(input_bag, Topic=[topic '_gt']));
    output_msgs = readMessages(select(input_bag, Topic=topic));

    % Extract timestamps
    input_timestamps = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec) * 1e-9, input_msgs);
    output_timestamps = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec) * 1e-9, output_msgs);
    input_timestamps = input_timestamps - input_timestamps(1);
    output_timestamps = output_timestamps - output_timestamps(1);

    % Extract torques
    input_efforts = cellfun(@(m) m.effort, input_msgs, UniformOutput=false);
    output_efforts = cellfun(@(m) m.effort, output_msgs, UniformOutput=false);

    % Convert to matrices (assuming fixed number of joints)
    n_joints = numel(input_efforts{ 1 });
    input_efforts_mat = cell2mat(cellfun(@(x) x(:)', input_efforts, UniformOutput=false));
    output_efforts_mat = cell2mat(cellfun(@(x) x(:)', output_efforts, UniformOutput=false));

    % Plot results
    set(groot, defaultAxesTickLabelInterpreter='latex');
    [~, name, ~] = fileparts(bag_file);
    figure(Name=name, Units="normalized", OuterPosition=[0 0 1 1], Color="white");
    n_cols = 2;
    n_rows = ceil(n_joints / n_cols);
    anim_computed = cell(n_joints, 1);
    anim_measured = cell(n_joints, 1);
    animate = lower(input("Do you want to animate plots? [y/n] ", "s")) == "y";
    for jj = 1 : n_joints
        ax = subplot(n_rows, n_cols, jj);
        if animate
            anim_computed{jj} = animatedline(Color='r', LineStyle='-', LineWidth=LW);
            anim_measured{jj} = animatedline(Color='k', LineStyle='--', LineWidth=LW);
        else
            plot(output_timestamps, output_efforts_mat(:, jj), 'r', LineWidth=LW, DisplayName="Computed"); hold on;
            plot(input_timestamps, input_efforts_mat(:, jj), 'k--', LineWidth=LW, DisplayName="Measured");
        end
        xlabel('Time [s]', Interpreter='latex', FontSize=FS);
        ylabel(['$\tau_' num2str(jj) '$ [Nm]'], Interpreter="latex", FontSize=FS);
        grid on;
        xlim([0, input_timestamps(end)]);
        ax.FontSize = FS;
    end
    sgtitle('Comparison of Ground Truth (black) and Computed (red) Torques');

    % Animate
    if animate
        pause(5);  % Pause 5 seconds
        nrtwp = 1;
        init_time = tic;
        while nrtwp ~= length(output_timestamps)
            nrtwp_old = nrtwp;
            start_computation_time = tic;

            % Search for the next waypoint to plot
            k = nrtwp;
            while k <= length(output_timestamps) && output_timestamps(k) <= toc(init_time)
                k = k + 1;
            end
            nrtwp = max(nrtwp, k - 1);

            % The sequence of new waypoints to plot
            seq = nrtwp_old : nrtwp;

            % Plot the new waypoints
            for jj = 1 : n_joints
                addpoints(anim_computed{jj}, output_timestamps(seq), output_efforts_mat(seq, jj));
                addpoints(anim_measured{jj}, input_timestamps(seq), input_efforts_mat(seq, jj));
            end

            % Pause
            computation_time = toc(start_computation_time);
            pause(1/30 - computation_time);
        end
    end

    % Create a table displaying the MAE for each joint
    mae_table = table(input_msgs{1}.name(:), ...
        mean(abs(output_efforts_mat - input_efforts_mat))', ...
        VariableNames={'Joint', 'MAE [Nm]'});
    disp(mae_table);
end

%% Export figures

if lower(input("Do you want to export figures? [y/n] ", "s")) == "y"
    export_figs(output_dir, 600, true);
end
