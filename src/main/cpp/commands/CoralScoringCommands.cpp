#include "commands/CoralScoringCommands.h"
#include <frc2/command/Commands.h>

using namespace frc2::cmd;

CoralScoringCommands::CoralScoringCommands(ElevatorSubsystem* elevator_subsystem, CoralSubsystem* coral_subsystem) {
    this->elevator_subsystem = elevator_subsystem;
    this->coral_subsystem = coral_subsystem;
}

CommandPtr CoralScoringCommands::score_l2() {
    return Sequence(
        coral_subsystem->intake(),
        coral_subsystem->pivot_to_shoot(),
        elevator_subsystem->pid_controll(ElevatorConstants::l2_height),
        coral_subsystem->outtake(),
        Wait(1_s),
        coral_subsystem->stop_intake()
    );
}

CommandPtr CoralScoringCommands::score_l3() {
    return Sequence(
        coral_subsystem->intake(),
        coral_subsystem->pivot_to_shoot(),
        elevator_subsystem->pid_controll(ElevatorConstants::l3_height),
        coral_subsystem->outtake(),
        Wait(1_s),
        coral_subsystem->stop_intake()
    );
}

CommandPtr CoralScoringCommands::coral_intake() {
    return Sequence(
        elevator_subsystem->to_bottom(),
        coral_subsystem->pivot_to_intake(),
        coral_subsystem->intake(),
        Wait(2_s),
        coral_subsystem->stop_intake()
    );
}