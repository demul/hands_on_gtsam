/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SelfCalibrationExample.cpp
 * @brief   Based on VisualSLAMExample, but with unknown (yet fixed) calibration.
 * @author  Frank Dellaert
 */

/*
 * See the detailed documentation in Visual SLAM.
 * The only documentation below with deal with the self-calibration.
 */

// 8 Points, 8 Pose, and motherfukin 3 Camera Example

// For loading the data
#include "SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// SFM-specific factors
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam_unstable/slam/ProjectionFactorPPPC.h>  // does calibration !

// Standard headers
#include <vector>

using namespace std;
using namespace gtsam;

Eigen::Matrix3d convert(double yaw, double pitch, double roll)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d qmat = q.matrix();
    return qmat;
}

int main(int argc, char* argv[])
{
    vector<Cal3DS2> intrinsics;
    intrinsics.emplace_back(45.0, 45.0, 0.0, 45.0, 45.0, 0.0001, 0.0001, 0.0001, 0.0001);
    intrinsics.emplace_back(40.0, 40.0, 0.0, 40.0, 40.0, 0.0001, 0.0001, 0.0001, 0.0001);
    intrinsics.emplace_back(55.0, 55.0, 0.0, 55.0, 55., 0.0001, 0.0001, 0.0001, 0.00010);

    vector<Pose3> transforms;
    transforms.emplace_back(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
    transforms.emplace_back(Rot3::Ypr(-M_PI / 18, 0, 0), Point3(-10, -10, 0));
    transforms.emplace_back(Rot3::Ypr(M_PI / 18, 0, 0), Point3(-10, 10, 0));

    if (intrinsics.size() != transforms.size())
    {
        cout << "The count of intrinsic and the count of transform are not same!" << endl;
        return -1;
    }

    vector<PinholeCamera<Cal3DS2>> rig_set;
    for (size_t camera_index = 0; camera_index < intrinsics.size(); camera_index++)
    {
        rig_set.emplace_back(transforms[camera_index], intrinsics[camera_index]);
    }

    // Create the set of ground-truth
    vector<Point3> points = createPoints();  // 8
    vector<Pose3> poses = createPoses();     // 8

    std::cout << poses[0] << std::endl;
    std::cout << convert(M_PI / 2, 0, -M_PI / 2) << std::endl;

    // Create the factor graph
    NonlinearFactorGraph graph;

    // Add a prior on pose x1.
    auto poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1), Vector3::Constant(0.2))
                                                      .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

    auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);
    for (size_t camera_index = 0; camera_index < rig_set.size(); camera_index++)
    {
        PinholeCamera<Cal3DS2>& rig = rig_set[camera_index];
        for (size_t pose_index = 0; pose_index < poses.size(); pose_index++)
        {
            PinholeCamera<Cal3DS2> camera(poses[pose_index].compose(rig.pose()), rig.calibration());
            for (size_t point_index = 0; point_index < points.size(); point_index++)
            {
                Point2 measurement = camera.project(points[point_index]);
                // The only real difference with the Visual SLAM example is that here we
                // use a different factor type, that also calculates the Jacobian with
                // respect to calibration
                graph.emplace_shared<ProjectionFactorPPPC<Pose3, Point3, Cal3DS2>>(
                    measurement,
                    measurementNoise,
                    Symbol('x', pose_index),
                    Symbol('r', camera_index),
                    Symbol('l', point_index),
                    Symbol('K', camera_index));
            }
        }
    }

    Values initialEstimate;

    auto pointNoise = noiseModel::Isotropic::Sigma(3, 100);
    for (size_t point_index = 0; point_index < points.size(); point_index++)
    {
        if (point_index == 1)
        {
            // Point3 tmp_point =
            //     points[point_index] +
            //     Point3(0.1 * pow(-1, point_index % 2), 0.1 * pow(-1, point_index % 3), 0.1 * pow(-1, point_index %
            //     5));
            // graph.addPrior(Symbol('l', point_index), tmp_point, pointNoise);
            graph.emplace_shared<NonlinearEquality<Point3>>(Symbol('l', point_index), points[point_index]);
        }
        else
        {
            graph.addPrior(Symbol('l', point_index), points[point_index], pointNoise);
            // graph.emplace_shared<NonlinearEquality<Point3>>(Symbol('l', point_index), points[point_index]);
        }
        // graph.emplace_shared<NonlinearEquality<Point3>>(Symbol('l', point_index), points[point_index]);
        initialEstimate.insert<Point3>(
            Symbol('l', point_index), points[point_index]
            // + Point3(0.1 * pow(-1, point_index % 2), 0.1 * pow(-1, point_index % 3), 0.1 * pow(-1, point_index % 5))
        );
    }

    for (size_t camera_index = 0; camera_index < rig_set.size(); camera_index++)
    {
        initialEstimate.insert(
            Symbol('K', camera_index), Cal3DS2(45.0, 45.0, 0.0, 45.0, 45.0, 0.0001, 0.0001, 0.0001, 0.0001));
        initialEstimate.insert(
            Symbol('r', camera_index),
            rig_set[camera_index].pose().compose(Pose3(
                Rot3::Rodrigues(
                    (M_PI / 90) * pow(-1, camera_index % 2),
                    (-M_PI / 90) * pow(-1, camera_index % 2),
                    (M_PI / 90) * pow(-1, camera_index % 2)),
                Point3(
                    (-2) * pow(-1, camera_index % 2),
                    (2) * pow(-1, camera_index % 2),
                    (-2) * pow(-1, camera_index % 2)))));
    }

    for (size_t pose_index = 0; pose_index < poses.size(); pose_index++)
    {
        initialEstimate.insert(
            Symbol('x', pose_index),
            poses[pose_index].compose(Pose3(
                Rot3::Rodrigues(
                    (0.5) * pow(-1, pose_index % 2), (0.5) * pow(-1, pose_index % 3), (0.4) * pow(-1, pose_index % 4)),
                Point3(
                    (0.05) * pow(-1, pose_index % 4),
                    (0.05) * pow(-1, pose_index % 3),
                    (0.05) * pow(-1, pose_index % 2)))));
    }

    /* Optimize the graph and print results */
    LevenbergMarquardtParams balgam;
    balgam.verbosityLM = LevenbergMarquardtParams::VerbosityLM::SUMMARY;
    Values result = LevenbergMarquardtOptimizer(graph, initialEstimate, balgam).optimize();
    result.print("Final results:\n");

    for (size_t camera_index = 0; camera_index < rig_set.size(); camera_index++)
    {
        cout << "=====GT Camera : " << camera_index << "======" << endl;
        cout << rig_set[camera_index].calibration() << endl;
        cout << rig_set[camera_index].pose() << endl;
    }

    for (const gtsam::Values::ConstKeyValuePair& key_value : result)
    {
        gtsam::Key key = key_value.key;

        // Parse the symbol from the key
        gtsam::Symbol symbol(key);  // Creating a symbol from the key

        // Print the parsed symbol and its associated value
        unsigned char x = 'x';
        if (x == symbol.chr())
            std::cout << "Parsed Symbol: " << symbol.chr() << symbol.index() << endl
                      << result.at<Pose3>(key) << std::endl;
    }

    return 0;
}
