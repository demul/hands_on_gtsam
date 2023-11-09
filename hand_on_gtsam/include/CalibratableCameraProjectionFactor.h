#pragma once

#include <gtsam/slam/GeneralSFMFactor.h>

namespace gtsam
{
template <class CALIBRATION>
class GeneralSFMFactor3 : public NoiseModelFactor4<Pose3, Pose3, Point3, CALIBRATION>
{
    GeneralSFMFactor3(
        const Point2& measured,
        const SharedNoiseModel& model,
        Key rigKey,
        Key sensorKey,
        Key landmarkKey,
        Key calibKey)
        : NoiseModelFactor4(model, rigKey, sensorKey, landmarkKey, calibKey), measured_(measured)
    {
    }

    CalibratableCameraProjectionFactor() : measured_(0.0, 0.0)
    {
    }

    ~CalibratableCameraProjectionFactor() override
    {
    }
}
}  // namespace gtsam