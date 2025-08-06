#include <iostream>
#include <iomanip>
#include "gtsam.hpp"

using namespace std;
using namespace gtsam;

int main() {
    cout << "========== GTSAM 2D Odometry Optimization ==========\n" << endl;

    // Create factor graph
    NonlinearFactorGraph graph;

    // Add prior on the first pose
    graph.add(PriorFactor<Pose2>(1,
        Pose2(0.0, 0.0, 0.0),
        noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1))));
    graph.add(BetweenFactor<Pose2>(1, 2,
        Pose2(2.0, 1.0, 0.0),
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1))));
    graph.add(BetweenFactor<Pose2>(2, 3,
        Pose2(2.0, 1.0, 0.0),
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1))));

    cout << "Factor graph created with " << graph.size() << " factors:\n";
    graph.print("  ");

    // Initial estimates
    Values initialEstimate;
    initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
    initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));

    cout << "\n Initial estimates:\n";
    initialEstimate.print("  ");

    // Calculate and print initial error
    double initialError = graph.error(initialEstimate);
    cout << std::fixed << std::setprecision(3);
    cout << "\n Initial total error: " << initialError << "\n";

    // Optimize using Levenberg-Marquardt
    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
    Values result = optimizer.optimize();

    // Final output
    cout << "\n Optimized poses:\n";
    result.print("  ");

    double finalError = graph.error(result);
    cout << "\n Final total error: " << finalError << "\n";

    // Query the marginals
    cout << std::fixed << std::setprecision(3);
    Marginals marginals(graph, result);
    for (size_t i = 1; i <= 3; ++i) {
        cout << "\nCovariance of pose " << i << ":\n" << marginals.marginalCovariance(i) << "\n";
    }

    cout << "\n========== Optimization Complete ==========\n";
    return 0;
}
