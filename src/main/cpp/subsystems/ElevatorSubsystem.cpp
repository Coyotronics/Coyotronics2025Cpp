#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkBase.h>
#include <rev/config/SparkMaxConfig.h>

ElevatorSubsystem::ElevatorSubsystem() {
    SparkMaxConfig motor_config{};

    motor_config.SmartCurrentLimit(30).SetIdleMode(SparkBaseConfig::IdleMode::kBrake);

    right_motor.Configure(motor_config, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    left_motor.Configure(motor_config, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    right_motor.GetEncoder().SetPosition(0);
    left_motor.GetEncoder().SetPosition(0);
}

void ElevatorSubsystem::Periodic() {
    stop();
    SmartDashboard::PutNumber("Elevator Height", get_height());
}

void ElevatorSubsystem::move_up() {
    right_motor.SetVoltage(-4_V);
    left_motor.SetVoltage(4_V);
}

void ElevatorSubsystem::move_down() {
    right_motor.SetVoltage(2_V);
    left_motor.SetVoltage(-2_V);
}

void ElevatorSubsystem::stop() {
    if (get_height() > 0.2) {
        right_motor.SetVoltage(-0.38_V);
        left_motor.SetVoltage(0.38_V);

        return;
    }

    right_motor.SetVoltage(0_V);
    left_motor.SetVoltage(0_V);
}

CommandPtr ElevatorSubsystem::pid_controll(double setpoint) {
    return Run([this, setpoint]() {
               double pid = calculate_pid(setpoint);

               right_motor.Set(-pid);
               left_motor.Set(pid);
           })
        .Until([this, setpoint]() {
            double pid = calculate_pid(setpoint);

            return std::round(pid) == 0.0;
        });
}

CommandPtr ElevatorSubsystem::to_bottom() {
    return Run([this]() {
               move_down();
           })
        .Until([this]() {
            return get_height() <= 0.05;
        }).OnlyIf([this]() {
            return get_height() > 0.05;
        });
}

double ElevatorSubsystem::calculate_pid(double setpoint) {
    double output = pid_controller.Calculate(get_height(), setpoint);
    output = std::clamp(output, -1.0, 1.0);

    return output;
}

double ElevatorSubsystem::get_height() {
    return left_motor.GetEncoder().GetPosition();
}