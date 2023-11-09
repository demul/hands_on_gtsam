/**
 * Reference link : https://github.com/borglab/gtsam/blob/develop/examples/OdometryExample.cpp
 # Reference author : Frank Dellaert
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

int main()
{
    // Create an empty nonlinear factor graph
    gtsam::NonlinearFactorGraph graph;

    // Add a prior on the first pose, setting it to the origin
    // A prior factor consists of a mean and a noise model (covariance matrix)
    gtsam::Pose2 prior_mean(0.0, 0.0, 0.0);  // prior at origin
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
    graph.addPrior(1, prior_mean, prior_noise);

    // Add odometry factors
    gtsam::Pose2 odometry(2.0, 0.0, 0.0);

    // For simplicity, we will use the same noise model for each odometry factor
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));

    // Create odometry (Between) factors between consecutive poses
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(1, 2, odometry, odometry_noise);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(2, 3, odometry, odometry_noise);
    graph.print("\nFactor Graph:\n");  // print

    // Create the data structure to hold the initialEstimate estimate to the solution
    // For illustrative purposes, these have been deliberately set to incorrect values
    gtsam::Values initial;
    initial.insert(1, gtsam::Pose2(0.0, 0.0, 0.0));
    initial.insert(2, gtsam::Pose2(2.0, 0.0, 0.0));
    initial.insert(3, gtsam::Pose2(4.0, 0.0, 0.0));
    // initial.print("\nInitial Estimate:\n");  // print

    // optimize using Levenberg-Marquardt optimization
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
    gtsam::Marginals marginals(graph, result);
    std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << std::endl;
    std::cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << std::endl;
    std::cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << std::endl;

    return 0;
}
