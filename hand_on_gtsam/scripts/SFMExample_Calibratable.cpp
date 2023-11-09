/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample_SmartFactor.cpp
 * @brief   A structure-from-motion problem on a simulated dataset, using smart projection factor
 * @author  Duy-Nguyen Ta
 * @author  Jing Dong
 * @author  Frank Dellaert
 */

// In GTSAM, measurement functions are represented as 'factors'.
// The factor we used here is SmartProjectionPoseFactor.
// Every smart factor represent a single landmark, seen from multiple cameras.
// The SmartProjectionPoseFactor only optimizes for the poses of a camera,
// not the calibration, which is assumed known.
#include <gtsam/slam/SmartProjectionFactor.h>

// For an explanation of these headers, see SFMExample.cpp
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "SFMdata.h"

using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3_S2> Camera;
typedef SmartProjectionFactor<Camera> SmartFactor;

/* ************************************************************************* */
int main(int argc, char* argv[])
{
    // Define the camera calibration parameters

    // Define the camera observation noise model
    auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

    // Create the set of ground-truth landmarks and poses
    vector<Point3> points = createPoints();
    vector<Pose3> poses = createPoses();

    // Create a factor graph
    NonlinearFactorGraph graph;

    std::vector<Camera> camera_set;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Cal3_S2 K(50, 50, 0.0, 50.0, 50.0);
        camera_set.emplace_back(poses[i], K);
    }
    // Cal3_S2 K6(50.5, 49.5, 0.0, 51.0, 49.0);
    // camera_set.emplace_back(poses[6], K6);
    // Cal3_S2 K7(40.5, 60.5, 0.0, 40.0, 55.0);
    // camera_set.emplace_back(poses[7], K7);

    // Simulated measurements from each camera pose, adding them to the factor graph
    for (size_t j = 0; j < points.size(); ++j)
    {
        // every landmark represent a single landmark, we use shared pointer to init the factor, and then insert
        // measurements.
        SmartFactor::shared_ptr smartfactor(new SmartFactor(measurementNoise));

        for (size_t i = 0; i < poses.size(); ++i)
        {
            // generate the 2D measurement
            Point2 measurement = camera_set[0].project(points[j]);

            // call add() function to add measurement into a single factor, here we need to add:
            //    1. the 2D measurement
            //    2. the corresponding camera's key
            //    3. camera noise model
            //    4. camera calibration
            smartfactor->add(measurement, i);
        }

        // insert the smart factor in the graph
        graph.push_back(smartfactor);
    }

    std::cout << "aaa" << std::endl;

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    auto noise = noiseModel::Diagonal::Sigmas(
        (Vector(11) << Vector3::Constant(1.1), Vector3::Constant(1.3), Vector3::Constant(1.0), Vector2::Constant(1.3))
            .finished());
    std::cout << "bbb" << std::endl;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        graph.addPrior(i, camera_set[i], noise);
    }

    graph.print("Factor Graph:\n");

    // Create the initial estimate to the solution
    // Intentionally initialize the variables off from the ground truth
    Values initialEstimate;
    for (size_t i = 0; i < poses.size(); ++i)
        initialEstimate.insert(i, camera_set[i]);
    initialEstimate.print("Initial Estimates:\n");

    // Optimize the graph and print results
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final results:\n");

    // A smart factor represent the 3D point inside the factor, not as a variable.
    // The 3D position of the landmark is not explicitly calculated by the optimizer.
    // To obtain the landmark's 3D position, we use the "point" method of the smart factor.
    Values landmark_result;
    for (size_t j = 0; j < points.size(); ++j)
    {
        // The graph stores Factor shared_ptrs, so we cast back to a SmartFactor first
        SmartFactor::shared_ptr smart = boost::dynamic_pointer_cast<SmartFactor>(graph[j]);
        if (smart)
        {
            // The output of point() is in boost::optional<Point3>, as sometimes
            // the triangulation operation inside smart factor will encounter degeneracy.
            boost::optional<Point3> point = smart->point(result);
            if (point)  // ignore if boost::optional return nullptr
                landmark_result.insert(j, *point);
        }
    }

    // landmark_result.print("Landmark results:\n");
    // cout << "final error: " << graph.error(result) << endl;
    // cout << "number of iterations: " << optimizer.iterations() << endl;

    return 0;
}
/* ************************************************************************* */
