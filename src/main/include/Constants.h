#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <numbers>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

namespace DriveConstants {
    constexpr units::meters_per_second_t max_speed = 4.8_mps;
    constexpr units::radians_per_second_t max_angular_speed{2 * std::numbers::pi};

    constexpr double direction_slew_rate = 1.2;
    constexpr double magnitude_slew_rate = 1.8;
    constexpr double rotational_slew_rate = 2.0;

    constexpr units::meter_t track_width = 0.6731_m;
    constexpr units::meter_t wheel_base = 0.6731_m;

    constexpr bool gyro_reversed = false;

    constexpr double front_left_chassis_angular_offset = -std::numbers::pi / 2;
    constexpr double front_right_chassis_angular_offset = 0;
    constexpr double back_left_chassis_angular_offset = std::numbers::pi;
    constexpr double back_right_chassis_angular_offset = std::numbers::pi / 2;

    constexpr int front_left_driving_can_id = 45;
    constexpr int back_left_driving_can_id = 12;
    constexpr int front_right_driving_can_id = 20;
    constexpr int back_right_driving_can_id = 10;

    constexpr int front_left_turning_can_id = 23;
    constexpr int back_left_turning_can_id = 13;
    constexpr int front_right_turning_can_id = 21;
    constexpr int back_right_turning_can_id = 22;
} // namespace DriveConstants

namespace ModuleConstants {
    constexpr int driving_motor_pinion_teeth = 14;

    constexpr double driving_motor_free_speed_rps = 5676.0 / 60;
    constexpr units::meter_t wheel_diameter = 0.0762_m;
    constexpr units::meter_t wheel_circumference = wheel_diameter * std::numbers::pi;

    constexpr double driving_motor_reduction = (45.0 * 22) / (driving_motor_pinion_teeth * 15);
    constexpr double drive_wheel_free_speed_rps = (driving_motor_free_speed_rps * wheel_circumference.value()) / driving_motor_reduction;
} // namespace ModuleConstants

namespace AutoConstants {
    constexpr auto max_speed = 3_mps;
    constexpr auto max_acceleration = 3_mps_sq;
    constexpr auto max_angular_speed = 3.142_rad_per_s;
    constexpr auto max_angular_acceleration = 3.142_rad_per_s_sq;

    constexpr double p_x_controller = 0.5;
    constexpr double p_y_controller = 0.5;
    constexpr double p_theta_controller = 0.5;

    extern const frc::TrapezoidProfile<units::radians>::Constraints theta_controller_constraints;
} // namespace AutoConstants

namespace OIConstants {
    constexpr int driver_controller_port = 0;
    constexpr int operator_controller_port = 1;
    constexpr double drive_deadband = 0.05;
} // namespace OIConstants

namespace ElevatorConstants {
    constexpr int right_motor_id = 5;
    constexpr int left_motor_id = 6;

    constexpr double p = 0.2;
    constexpr double i = 0.0;
    constexpr double d = 0.0;

    constexpr double l2_height = 39.3;
    constexpr double l3_height = 67.0;
} // namespace ElevatorConstants

namespace CoralConstants {
    constexpr int intake_motor_id = 36;
    constexpr int pivot_motor_id = 7;

    constexpr double pivot_up_stop = 0.5;
    constexpr double pivot_down_stop = 0.3;
} // namespace CoralConstants