#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>

using namespace rev::spark;
using namespace frc;

class MAXSwerveModule {
public:
    MAXSwerveModule(int driving_can_id, int turning_can_id, double chassis_angular_offset);
    SwerveModuleState get_state() const;
    SwerveModulePosition get_position() const;
    void set_desired_state(SwerveModuleState desired_state);
    void reset_encoders();

private:
    SparkMax driving_spark_max;
    SparkMax turning_spark_max;

    SparkRelativeEncoder driving_encoder = driving_spark_max.GetEncoder();
    SparkAbsoluteEncoder turning_encoder = turning_spark_max.GetAbsoluteEncoder();

    SparkClosedLoopController driving_pid_controller = driving_spark_max.GetClosedLoopController();
    SparkClosedLoopController turning_pid_controller = turning_spark_max.GetClosedLoopController();

    SwerveModuleState state{units::meters_per_second_t{0.0}, frc::Rotation2d()};
    double chassis_angular_offset = 0;
};