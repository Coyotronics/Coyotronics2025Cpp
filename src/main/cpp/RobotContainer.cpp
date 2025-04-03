// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <utility>

RobotContainer::RobotContainer() {
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    drive_subsystem.SetDefaultCommand(frc2::RunCommand(
        [this] {
            drive_subsystem.drive(
                -units::meters_per_second_t{frc::ApplyDeadband(
                    driver_controller.GetLeftY(), OIConstants::drive_deadband)},
                -units::meters_per_second_t{frc::ApplyDeadband(
                    driver_controller.GetLeftX(), OIConstants::drive_deadband)},
                -units::radians_per_second_t{frc::ApplyDeadband(
                    driver_controller.GetRightX(), OIConstants::drive_deadband)},
                false);
        },
        {&drive_subsystem}));
    
    button_board.Button(3).OnTrue(coral_scoring_commands.score_l2().OnlyIf([this] { return !button_board.Button(2).Get(); }));
    button_board.Button(4).OnTrue(coral_scoring_commands.score_l2());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No autonomous command configured");
}
