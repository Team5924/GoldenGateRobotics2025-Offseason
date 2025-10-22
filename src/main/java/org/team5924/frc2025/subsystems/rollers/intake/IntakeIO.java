package org.team5924.frc2025.subsystems.rollers.intake;

import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIO;

public interface IntakeIO extends GenericRollerSystemIO {
    
  @AutoLog
  abstract class IntakeIOInputs extends GenericRollerSystemIOInputs {}

  default void runVolts(double intakeVolts, double alignerVolts) {}
}
