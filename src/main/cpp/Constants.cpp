#include "Constants.h"

namespace AutoConstants {
    const frc::TrapezoidProfile<units::radians>::Constraints theta_controller_constraints{max_angular_speed, max_angular_acceleration};
}