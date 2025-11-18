/*
 * Climber.java
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

package org.team5924.frc2025.subsystems.climber;

import com.ctre.phoenix6.controls.CoastOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
  public enum ClimberState {
    STOPPED(
        new LoggedTunableNumber("Climber/StoppedAngle", Math.toRadians(98.0)),
        new LoggedTunableNumber("Climber/StoppedVoltage", 0.0)),

    LINEUP( // lines up before grabbing onto cage
        new LoggedTunableNumber("Climber/LineupForwardAngle", Math.toRadians(0.0)),
        new LoggedTunableNumber("Climber/LineupForwardVoltage", -1.0)), // -4
    HANGING( // after grabbing onto cage, goes back up
        new LoggedTunableNumber("Climber/HangingAngle", Math.toRadians(99.0)),
        new LoggedTunableNumber("Climber/HangingVoltage", 1.0)); // 12

    public final LoggedTunableNumber angle;
    public final LoggedTunableNumber forwardsVoltage;

    ClimberState(LoggedTunableNumber angle, LoggedTunableNumber forwardsVoltage) {
      this.angle = angle;
      this.forwardsVoltage = forwardsVoltage;
    }
  }

  // private static final double PASS_ANGLE_CHECK = 0.0;

  private final CoastOut coastNeutralRequest = new CoastOut();

  private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final LoggedTunableNumber CLIMBER_POSITION_TOLERANCE =
      new LoggedTunableNumber("Climber/Position Tolerance", Math.toRadians(2.0));

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    handleCurrentState();
    io.updateInputs(inputs);

    Logger.recordOutput("Climber/CurrentState", RobotState.getInstance().getClimberState());
    Logger.recordOutput("Climber/CurrentRads", inputs.climbPositionRads);
    Logger.recordOutput("Climber/TargetRads", RobotState.getInstance().getClimberState().angle);
    Logger.recordOutput(
        "Climber/TargetVoltage", RobotState.getInstance().getClimberState().forwardsVoltage);
    Logger.recordOutput("Climber/AtGoal", atGoal());
  }

  public void handleCurrentState() {
    ClimberState state = RobotState.getInstance().getClimberState();
    switch (state) {
      case STOPPED -> {
        if (DriverStation.isDisabled()) {
          io.setClimbControl(coastNeutralRequest);
        } else {
          io.disableClimbTalon();
        }
        io.disableGrabTalon();
      }
      case LINEUP -> {
        if (atGoal()) {
          io.disableClimbTalon();
        } else {
          io.runClimbVolts(state.forwardsVoltage.getAsDouble());
        }
        io.runGrabVolts(12.0);
      }
      case HANGING -> {
        if (atGoal()) {
          io.disableClimbTalon();
        } else {
          io.runClimbVolts(state.forwardsVoltage.getAsDouble());
        }
        io.disableGrabTalon();
      }
    }
  }

  public void setState(ClimberState newState) {
    if (Constants.Climber.REQUIRE_AT_GOAL) {
      RobotState.getInstance().setClimberState(newState);
      return;
    }

    switch (newState) {
      case LINEUP, STOPPED -> RobotState.getInstance().setClimberState(newState);
      case HANGING -> {
        if (RobotState.getInstance().getClimberState().equals(ClimberState.LINEUP) && atGoal()) {
          RobotState.getInstance().setClimberState(newState);
        }
      }
      default -> {}
    }
  }

  // * simply return true because not using a canrange */
  public boolean isHoldingCage() {
    return true;
  }

  public boolean atGoal() {
    double goal = clamp(RobotState.getInstance().getClimberState().angle.getAsDouble());
    double distanceToGoal = Math.abs(inputs.climbPositionRads - goal);
    return distanceToGoal <= CLIMBER_POSITION_TOLERANCE.getAsDouble();
  }

  private static double clamp(double angle) {
    return MathUtil.clamp(angle, Constants.Climber.MIN_RADS, Constants.Climber.MAX_RADS);
  }
}
