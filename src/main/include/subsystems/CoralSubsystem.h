#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

using namespace frc2;
using namespace frc;
using namespace rev::spark;

class CoralSubsystem : public SubsystemBase {
public:
    CoralSubsystem();
    void Periodic() override;

    CommandPtr pivot_to_intake();
    CommandPtr pivot_to_shoot();

    CommandPtr intake();
    CommandPtr outtake();
    CommandPtr stop_intake();

private:
    SparkMax pivot_motor{CoralConstants::pivot_motor_id, SparkMax::MotorType::kBrushless};
    SparkMax intake_motor{CoralConstants::intake_motor_id, SparkMax::MotorType::kBrushless};

    double get_pivot_position();
};