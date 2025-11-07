/*
 * Intake.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.rollers.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  public enum IntakeState {
    IN( // used for ground intake
        new LoggedTunableNumber("Intake/IntakeMotor/InVoltage", -6.0),
        new LoggedTunableNumber("Intake/AlignerMotor/InVoltage", -8.0)),
    SLOW_IN( // used for source intake (?) and holding inside
        new LoggedTunableNumber("Intake/IntakeMotor/SlowInVoltage", -2.0),
        new LoggedTunableNumber("Intake/AlignerMotor/SlowInVoltage", -3.0)),
    TROUGH_OUT(
        new LoggedTunableNumber("Intake/IntakeMotor/TroughOutVoltage", 3.25),
        new LoggedTunableNumber("Intake/AlignerMotor/TroughOutVoltage", 0.0)),
    OUT(
        new LoggedTunableNumber("Intake/IntakeMotor/OutVoltage", 8.0),
        new LoggedTunableNumber("Intake/AlignerMotor/OutVoltage", 0.0)),
    OFF(
        new LoggedTunableNumber("Intake/IntakeMotor/OffVoltage", 0.0),
        new LoggedTunableNumber("Intake/AlignerMotor/OffVoltage", 0.0)),
    ALGAE_MODE_IDLE(
        new LoggedTunableNumber("Intake/IntakeMotor/AlgaeModeIdleVoltage", 0.0),
        new LoggedTunableNumber("Intake/AlignerMotor/AlgaeModeIdleVoltage", 0.0)),
    OPERATOR_CONTROL(
        new LoggedTunableNumber("Intake/IntakeMotor/OperatorControlVoltage", 0.0),
        new LoggedTunableNumber("Intake/AlignerMotor/OperatorControlVoltage", 0.0));

    public final DoubleSupplier intakeVoltage;
    public final DoubleSupplier alignerVoltage;

    IntakeState(LoggedTunableNumber intakeVoltage, LoggedTunableNumber alignerVoltage) {
      this.intakeVoltage = intakeVoltage;
      this.alignerVoltage = alignerVoltage;
    }
  }

  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // updateState();

    io.runVolts(
        RobotState.getInstance().getIntakeState().intakeVoltage.getAsDouble(),
        RobotState.getInstance().getIntakeState().alignerVoltage.getAsDouble());

    Logger.recordOutput("RobotState/IntakeState", RobotState.getInstance().getIntakeState());
  }

  public boolean isOperatorControlling() {
    IntakeState currentState = RobotState.getInstance().getIntakeState();
    return currentState.equals(IntakeState.OUT) || currentState.equals(IntakeState.TROUGH_OUT);
  }

  public void updateState() {
    if (isOperatorControlling()) return;
    if (false) // !inputs.beamBreakUnbroken)
    RobotState.getInstance().setIntakeState(IntakeState.SLOW_IN);
    else RobotState.getInstance().setIntakeState(IntakeState.OFF);
  }

  public void setGoalState(IntakeState state) {
    RobotState.getInstance().setIntakeState(state);
  }
}
