#include "subsystems/CoralSubsystem.h"

#include <rev/config/SparkMaxConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

using namespace frc2::cmd;

CoralSubsystem::CoralSubsystem() {
    SparkMaxConfig pivot_config;
    pivot_config.SmartCurrentLimit(40).SetIdleMode(SparkMaxConfig::IdleMode::kBrake);

    SparkMaxConfig intake_config;
    intake_config.SmartCurrentLimit(40).SetIdleMode(SparkMaxConfig::IdleMode::kCoast);

    pivot_motor.Configure(pivot_config, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    intake_motor.Configure(intake_config, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
}

void CoralSubsystem::Periodic() {
    SmartDashboard::PutNumber("Coral Pivot Position", get_pivot_position());
}

double CoralSubsystem::get_pivot_position() {
    return pivot_motor.GetAbsoluteEncoder().GetPosition();
}

CommandPtr CoralSubsystem::pivot_to_intake() {
    return Parallel(
        RunOnce([this] {
            pivot_motor.Set(0.2);
        }),
        WaitUntil([this] {
            return get_pivot_position() >= CoralConstants::pivot_up_stop;
        }).AndThen([this] {
            pivot_motor.Set(0);
        })
    ).OnlyIf([this] {
        return get_pivot_position() < CoralConstants::pivot_up_stop;
    });
}

CommandPtr CoralSubsystem::pivot_to_shoot() {
    return Parallel(
        RunOnce([this] {
            pivot_motor.Set(-0.15);
        }),
        WaitUntil([this] {
            return get_pivot_position() <= CoralConstants::pivot_down_stop;
        }).AndThen([this] {
            pivot_motor.Set(0);
        })
    ).OnlyIf([this] {
        return get_pivot_position() > CoralConstants::pivot_down_stop;
    });
}

CommandPtr CoralSubsystem::intake() {
    return RunOnce([this] {
        intake_motor.Set(-0.25);
    });
}

CommandPtr CoralSubsystem::outtake() {
    return RunOnce([this] {
        intake_motor.Set(0.25);
    });
}

CommandPtr CoralSubsystem::stop_intake() {
    return RunOnce([this] {
        intake_motor.Set(0);
    });
}