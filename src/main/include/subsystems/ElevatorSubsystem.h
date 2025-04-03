#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>

using namespace frc2;
using namespace frc;
using namespace rev::spark;

class ElevatorSubsystem : public SubsystemBase {
public:
    ElevatorSubsystem();
    void Periodic() override;

    CommandPtr pid_controll(double setpoint);
    CommandPtr to_bottom();

    void stop();
    void move_up();
    void move_down();

private:
    SparkMax right_motor{ElevatorConstants::right_motor_id, SparkMax::MotorType::kBrushless};
    SparkMax left_motor{ElevatorConstants::left_motor_id, SparkMax::MotorType::kBrushless};

    PIDController pid_controller{ElevatorConstants::p, ElevatorConstants::i, ElevatorConstants::d};

    double get_height();
    double calculate_pid(double setpoint);
};