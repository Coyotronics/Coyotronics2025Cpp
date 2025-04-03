#pragma once

#include "Constants.h"
#include "subsystems/MAXSwerveModule.h"

#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/ADIS16470_IMU.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

using namespace frc2;
using namespace frc;
using namespace ctre::phoenix6::hardware;
using namespace DriveConstants;

class DriveSubsystem : public SubsystemBase {
public:
    DriveSubsystem();
    void Periodic() override;

    void drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rot, bool field_relative);
    void set_module_states(wpi::array<SwerveModuleState, 4> desired_states);
    void reset_odometry(Pose2d pose);

    void setx();

    void reset_encoders();
    void zero_heading();

    units::degree_t get_heading();
    units::radian_t get_angle();
    double get_turn_rate();
    Pose2d get_pose();

    SwerveDriveKinematics<4> drive_kinematics{
        Translation2d{DriveConstants::wheel_base / 2, DriveConstants::track_width / 2},
        Translation2d{DriveConstants::wheel_base / 2, -DriveConstants::track_width / 2},
        Translation2d{-DriveConstants::wheel_base / 2, DriveConstants::track_width / 2},
        Translation2d{-DriveConstants::wheel_base / 2, -DriveConstants::track_width / 2}};

private:
    MAXSwerveModule front_left{front_left_driving_can_id, front_left_turning_can_id, front_left_chassis_angular_offset};
    MAXSwerveModule front_right{front_right_driving_can_id, front_right_turning_can_id, front_right_chassis_angular_offset};
    MAXSwerveModule back_left{back_left_driving_can_id, back_left_turning_can_id, back_left_chassis_angular_offset};
    MAXSwerveModule back_right{back_right_driving_can_id, back_right_turning_can_id, back_right_chassis_angular_offset};

    double current_rotation = 0.0;
    double current_translation_dir = 0.0;
    double current_translation_mag = 0.0;

    Pigeon2 gyro{34, "rio"};

    SwerveDriveOdometry<4> odometry{drive_kinematics, Rotation2d(get_angle()), {front_left.get_position(), front_right.get_position(), back_left.get_position(), back_right.get_position()}, Pose2d{}};
    SwerveDrivePoseEstimator<4> pose_estimator{
        drive_kinematics,
        gyro.GetRotation2d(),
        {front_left.get_position(), front_right.get_position(), back_left.get_position(), back_right.get_position()},
        Pose2d{},
        {0.05, 0.05, 5 * std::numbers::pi / 180},
        {0.5, 0.5, 30 * std::numbers::pi / 180}};

    bool use_localization = true;
};