#include "Configs.h"
#include "subsystems/MAXSwerveModule.h"

using namespace rev::spark;
using namespace Configs;
using namespace frc;

MAXSwerveModule::MAXSwerveModule(int driving_can_id, int turning_can_id, double chassis_angular_offset) : driving_spark_max(driving_can_id, SparkMax::MotorType::kBrushless), turning_spark_max(turning_can_id, SparkMax::MotorType::kBrushless) {
    driving_spark_max.Configure(SwerveConfigs::DrivingConfig(), SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    turning_spark_max.Configure(SwerveConfigs::TurningConfig(), SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    MAXSwerveModule::chassis_angular_offset = chassis_angular_offset;
    state.angle = frc::Rotation2d(units::radian_t{turning_encoder.GetPosition()});
    driving_encoder.SetPosition(0);
}

SwerveModuleState MAXSwerveModule::get_state() const {
    return {units::meters_per_second_t{driving_encoder.GetVelocity()}, units::radian_t{turning_encoder.GetPosition() - chassis_angular_offset}};
}

SwerveModulePosition MAXSwerveModule::get_position() const {
    return {units::meter_t{driving_encoder.GetPosition()}, units::radian_t{turning_encoder.GetPosition() - chassis_angular_offset}};
}

void MAXSwerveModule::set_desired_state(SwerveModuleState desired_state) {
    SwerveModuleState corrected_desired_state{};
    corrected_desired_state.speed = desired_state.speed;
    corrected_desired_state.angle = desired_state.angle + Rotation2d(units::radian_t{chassis_angular_offset});

    corrected_desired_state.Optimize(Rotation2d(units::radian_t{turning_encoder.GetPosition()}));

    driving_pid_controller.SetReference((double)corrected_desired_state.speed, SparkMax::ControlType::kVelocity);
    turning_pid_controller.SetReference(corrected_desired_state.angle.Radians().value(), SparkMax::ControlType::kPosition);

    MAXSwerveModule::state = desired_state;
}

void MAXSwerveModule::reset_encoders() {
    driving_encoder.SetPosition(0);
}