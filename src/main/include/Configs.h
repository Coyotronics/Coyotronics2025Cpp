#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

using namespace rev::spark;

namespace Configs {
    class SwerveConfigs {
    public:
        static SparkMaxConfig &DrivingConfig() {
            static SparkMaxConfig driving_config{};

            double driving_factor = ModuleConstants::wheel_diameter.value() *
                                    std::numbers::pi /
                                    ModuleConstants::driving_motor_reduction;
            double driving_velocity_feed_forward =
                1 / ModuleConstants::drive_wheel_free_speed_rps;

            driving_config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
                .SmartCurrentLimit(50);
            driving_config.encoder
                .PositionConversionFactor(driving_factor)
                .VelocityConversionFactor(driving_factor / 60.0);
            driving_config.closedLoop
                .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                .Pid(0.04, 0, 0)
                .VelocityFF(driving_velocity_feed_forward)
                .OutputRange(-1, 1);

            return driving_config;
        }

        static SparkMaxConfig &TurningConfig() {
            static SparkMaxConfig turning_config{};

            double turning_factor = 2 * std::numbers::pi;

            turning_config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
                .SmartCurrentLimit(20);
            turning_config.absoluteEncoder
                .Inverted(true)
                .PositionConversionFactor(turning_factor)
                .VelocityConversionFactor(turning_factor / 60.0);
            turning_config.closedLoop
                .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
                .Pid(1, 0, 0)
                .OutputRange(-1, 1)
                .PositionWrappingEnabled(true)
                .PositionWrappingInputRange(0, turning_factor);

            return turning_config;
        }
    };
} // namespace Configs