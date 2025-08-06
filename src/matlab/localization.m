clear all; close all; clc;

%% Create a factor graph
graph = gtsam.NonlinearFactorGraph;

%% Add a prior on pose 1
priorNoise = gtsam.noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(gtsam.PriorFactorPose2(1, gtsam.Pose2(0, 0, 0), priorNoise));

%% Add odometry factors
model = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(gtsam.BetweenFactorPose2(1, 2, gtsam.Pose2(2, 0, 0), model));
graph.add(gtsam.BetweenFactorPose2(2, 3, gtsam.Pose2(2, 0, pi/2), model));
graph.add(gtsam.BetweenFactorPose2(3, 4, gtsam.Pose2(2, 0, pi/2), model));
graph.add(gtsam.BetweenFactorPose2(4, 5, gtsam.Pose2(2, 0, pi/2), model));

%% Add loop closure (pose constraint)
graph.add(gtsam.BetweenFactorPose2(5, 2, gtsam.Pose2(2, 0, pi/2), model));

%% Initial estimates
initial = gtsam.Values();
initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2));
initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2));
initial.insert(3, gtsam.Pose2(4.1, 0.1, pi/2 - 0.2));
initial.insert(4, gtsam.Pose2(4.0, 2.0, pi - 0.2));
initial.insert(5, gtsam.Pose2(2.1, 2.1, -pi/2 - 0.2));
initial.print('Initial Estimates: ');

%% Compute initial marginal covariances
initial_marginals = gtsam.Marginals(graph, initial);
for i = 1:5
    key = i;
    P = initial_marginals.marginalCovariance(key);
    fprintf('Initial marginal covariance for pose %d:\n', i);
    disp(P);
end

%% Optimize using Levenberg-Marquardt optimization
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimize();

%% Print result
result.print('Final Result: ');

%% Compute and print the error before and after optimization    
initialError = graph.error(initial);
finalError = graph.error(result);
fprintf('Initial error: %.3f\n', initialError);
fprintf('Final error: %.3f\n', finalError);

%% Calculate final marginal covariances
final_marginals = gtsam.Marginals(graph, result);
for i = 1:5
    key = i;
    P = final_marginals.marginalCovariance(key);
    fprintf('Marginal covariance for pose %d:\n', i);
    disp(P);
end

%% Plot results
fig = figure('Visible', 'off', 'Position', [50, 50, 1600, 800]);

% Before optimization
subplot(1,2,1); hold on;
gtsam.plot2DTrajectory(initial, 'b-', initial_marginals);
title('Before Optimization', 'FontSize', 14);
xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12);
axis equal; grid on; xlim([-1, 5]); ylim([-2, 5]);

% After optimization
subplot(1,2,2); hold on;
gtsam.plot2DTrajectory(result, 'r-', final_marginals);
title('After Optimization', 'FontSize', 14);
xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12);
axis equal; grid on; xlim([-1, 5]); ylim([-2, 5]);

%% Save the plot
output_dir = '../../results/matlab/';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
exportgraphics(fig, fullfile(output_dir, 'localization_trajectory.pdf'), 'ContentType', 'vector');
disp(['Plot saved to ', fullfile(output_dir, 'localization_trajectory.pdf')]);
close(fig);

%% Save the graph and results to a file
gtsam.writeG2o(graph, result, fullfile(output_dir, 'localization_graph.g2o'));
disp(['Graph and results saved to ', fullfile(output_dir, 'localization_graph.g2o')]);