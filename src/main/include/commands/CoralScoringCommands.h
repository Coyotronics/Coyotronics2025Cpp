#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/CoralSubsystem.h"

using namespace frc2;

class CoralScoringCommands {
public:
    CoralScoringCommands(ElevatorSubsystem* elevator_subsystem, CoralSubsystem* coral_subsystem);

    CommandPtr score_l2();
    CommandPtr score_l3();

    CommandPtr coral_intake();

private:
    ElevatorSubsystem* elevator_subsystem;
    CoralSubsystem* coral_subsystem;
};