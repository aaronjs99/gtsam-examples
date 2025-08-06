datafile = gtsam.findExampleDataFile('sphere2500.txt');
model = gtsam.noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);

%% Initialize graph, initial estimate, and odometry noise
[graph, unconstrained] = gtsam.load3D(datafile, model, true, 2500);
graph.add(gtsam.NonlinearEqualityPose3(0, gtsam.Pose3()));
unconstrained_marginals = gtsam.Marginals(graph, unconstrained);

%% Read again, now with all constraints, and optimize
[graph, constrained] = gtsam.load3D(datafile, model, false, 2500);
graph.add(gtsam.NonlinearEqualityPose3(0, gtsam.Pose3()));
constrained_marginals = gtsam.Marginals(graph, constrained);

optimizer = gtsam.LevenbergMarquardtOptimizer(graph, unconstrained);
result = optimizer.optimizeSafely();

%% Plot results
fig = figure('Visible', 'off', 'Position', [50, 50, 1600, 800]);

% Before optimization
subplot(1,2,1); hold on;
gtsam.plot3DTrajectory(unconstrained, unconstrained_marginals, 'g-');
title('Unconstrained Trajectory', 'FontSize', 14);
xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12);
axis equal; grid on;

subplot(1,2,2); hold on;
gtsam.plot3DTrajectory(result, constrained_marginals, 'r-');
title('Constrained Trajectory', 'FontSize', 14);
xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12);
axis equal; grid on;

%% Save the plot
output_dir = '../../results/matlab/';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
exportgraphics(fig, fullfile(output_dir, 'poseslam3d_trajectory.pdf'), 'ContentType', 'vector');
disp(['Plot saved to ', fullfile(output_dir, 'poseslam3d_trajectory.pdf')]);
close(fig);

%% Save the graph and results to a file
gtsam.writeG2o(graph, result, fullfile(output_dir, 'poseslam3d_graph.g2o'));
disp(['Graph and results saved to ', fullfile(output_dir, 'poseslam3d_graph.g2o')]);