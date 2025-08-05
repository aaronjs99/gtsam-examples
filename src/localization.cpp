#include <iostream>
#include <iomanip>
#include "gtsam.hpp"

using namespace std;
using namespace gtsam;

int main() {
    cout << "========== GTSAM 2D Localization ==========\n" << endl;

    // Create factor graph
    NonlinearFactorGraph graph;

    // Add prior on the first pose
    graph.add(PriorFactor<Pose2>(1,
        Pose2(0.0, 0.0, 0.0),
        noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1))));

    // Add odometry factors
    graph.add(BetweenFactor<Pose2>(1, 2,
        Pose2(2.0, 0.0, 0.0),
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1))));
    graph.add(BetweenFactor<Pose2>(2, 3,
        Pose2(2.0, 0.0, M_PI/2),
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1))));
    graph.add(BetweenFactor<Pose2>(3, 4,
        Pose2(0.0, 2.0, M_PI/2),
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1))));
    graph.add(BetweenFactor<Pose2>(4, 5,
        Pose2(-2.0, 0.0, M_PI/2),
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1))));
    graph.add(BetweenFactor<Pose2>(5, 6,
        Pose2(0.0, -2.0, M_PI/2),
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1))));

    // Add loop closure factor
    graph.add(BetweenFactor<Pose2>(6, 1,
        Pose2(0.0, 0.0, M_PI/2),
        noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05))));

    // Initial estimate
    Values initialEstimate;
    initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
    initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI/2 + 0.1));
    initialEstimate.insert(4, Pose2(4.0, 2.1, M_PI + 0.1));
    initialEstimate.insert(5, Pose2(2.1, 2.0, -M_PI/2 + 0.1));
    initialEstimate.insert(6, Pose2(2.0, -0.1, 0.0));

    // Optimize using Levenberg-Marquardt
    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
    Values result = optimizer.optimize();

    // Print results
    cout << "Final Result:\n";
    result.print("Final Poses:\n");
    
    cout << "\n========== Localization Complete ==========\n";
    return 0;
}
