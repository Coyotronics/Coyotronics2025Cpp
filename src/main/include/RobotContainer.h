// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "commands/CoralScoringCommands.h"
#include "Constants.h"

class RobotContainer {
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();

    CommandXboxController driver_controller{OIConstants::driver_controller_port};
    CommandJoystick button_board{OIConstants::operator_controller_port};

    DriveSubsystem drive_subsystem;
    ElevatorSubsystem elevator_subsystem;
    CoralSubsystem coral_subsystem;

    CoralScoringCommands coral_scoring_commands{&elevator_subsystem, &coral_subsystem};
};
