#include "subsystems/DriveSubsystem.h"

#include <frc/DriverStation.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "LimelightHelpers.h"

using namespace DriveConstants;
using namespace frc;

DriveSubsystem::DriveSubsystem() {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}

void DriveSubsystem::Periodic() {
    bool reject_update = false;

    if (use_localization) {
        pose_estimator.Update(gyro.GetRotation2d(),
                              {front_left.get_position(), front_right.get_position(), back_left.get_position(), back_right.get_position()});

        LimelightHelpers::SetRobotOrientation("limelight", pose_estimator.GetEstimatedPosition().Rotation().Degrees().value(), 0, 0, 0, 0, 0);

        LimelightHelpers::PoseEstimate mega_tag2;

        std::optional<DriverStation::Alliance> alliance = DriverStation::GetAlliance();
        if (alliance.has_value()) {
            switch (alliance.value()) {
            case DriverStation::Alliance::kRed:
                mega_tag2 = LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("limelight");
                break;

            case DriverStation::Alliance::kBlue:
                mega_tag2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
                break;

            default:
                break;
            }
        }

        if (abs(get_turn_rate()) > 720) {
            reject_update = false;
        }

        if (mega_tag2.tagCount == 0) {
            reject_update = true;
        }

        if (!reject_update) {
            pose_estimator.SetVisionMeasurementStdDevs({0.7, 0.7, 9999999});
            pose_estimator.AddVisionMeasurement(mega_tag2.pose, mega_tag2.timestampSeconds);
        }
    } else {
        odometry.Update(Rotation2d(get_angle()),
                        {front_left.get_position(), back_left.get_position(),
                         front_right.get_position(), back_right.get_position()});
    }
}

void DriveSubsystem::drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rot, bool field_relative) {
    units::meters_per_second_t x_speed_delivered = x_speed.value() * DriveConstants::max_speed;
    units::meters_per_second_t y_speed_delivered = y_speed.value() * DriveConstants::max_speed;
    units::radians_per_second_t rot_delivered = rot.value() * DriveConstants::max_angular_speed;

    auto states = drive_kinematics.ToSwerveModuleStates(
        field_relative
            ? ChassisSpeeds::FromFieldRelativeSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered, Rotation2d(get_angle()))
            : ChassisSpeeds{x_speed_delivered, y_speed_delivered, rot_delivered});

    drive_kinematics.DesaturateWheelSpeeds(&states, DriveConstants::max_speed);

    auto [fl, fr, bl, br] = states;

    front_left.set_desired_state(fl);
    front_right.set_desired_state(fr);
    back_left.set_desired_state(bl);
    back_right.set_desired_state(br);
}

void DriveSubsystem::setx() {
    front_left.set_desired_state(SwerveModuleState{0_mps, Rotation2d{45_deg}});
    front_right.set_desired_state(SwerveModuleState{0_mps, Rotation2d{-45_deg}});
    back_left.set_desired_state(SwerveModuleState{0_mps, Rotation2d{-45_deg}});
    back_right.set_desired_state(SwerveModuleState{0_mps, Rotation2d{45_deg}});
}

void DriveSubsystem::set_module_states(wpi::array<SwerveModuleState, 4> desiredStates) {
    drive_kinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::max_speed);

    front_left.set_desired_state(desiredStates[0]);
    front_right.set_desired_state(desiredStates[1]);
    back_left.set_desired_state(desiredStates[2]);
    back_right.set_desired_state(desiredStates[3]);
}

void DriveSubsystem::reset_encoders() {
    front_left.reset_encoders();
    front_right.reset_encoders();
    back_left.reset_encoders();
    back_right.reset_encoders();
}

units::degree_t DriveSubsystem::get_heading() {
    return Rotation2d(get_angle()).Degrees();
}

void DriveSubsystem::zero_heading() { gyro.Reset(); }

double DriveSubsystem::get_turn_rate() {
    return gyro.GetAngularVelocityZWorld().GetValueAsDouble() * (gyro_reversed ? -1.0 : 1.0);
}

Pose2d DriveSubsystem::get_pose() {
    if (use_localization) {
        return pose_estimator.GetEstimatedPosition();
    } else {
        return odometry.GetPose();
    }
}

void DriveSubsystem::reset_odometry(Pose2d pose) {
    if (use_localization) {
        pose_estimator.ResetPosition(
            gyro.GetRotation2d(),
            {front_left.get_position(), front_right.get_position(), back_left.get_position(), back_right.get_position()},
            pose);
    } else {
        odometry.ResetPosition(get_heading(),
                               {front_left.get_position(), front_right.get_position(), back_left.get_position(), back_right.get_position()},
                               pose);
    }
}

units::radian_t DriveSubsystem::get_angle() {
    return units::radian_t{gyro.GetYaw().GetValueAsDouble()};
}