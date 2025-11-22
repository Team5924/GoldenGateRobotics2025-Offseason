/*
 * ArmPivot.java
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

package org.team5924.frc2025.subsystems.pivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.Elastic;
import org.team5924.frc2025.util.Elastic.Notification;
import org.team5924.frc2025.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class ArmPivot extends SubsystemBase {
  private final ArmPivotIO io;

  private final LoggedTunableNumber armPivotTolerance =
      new LoggedTunableNumber("ArmPivotToleranceRads", 0); // Subject to change
  private final ArmPivotIOInputsAutoLogged inputs = new ArmPivotIOInputsAutoLogged();

  public enum ArmPivotState {
    // TODO: Tune all these values
    ARM_UP(new LoggedTunableNumber("ArmPivotLowStowRads", 0)),
    ALGAE_GROUND(new LoggedTunableNumber("ArmPivotAlgaeGroundRads", 0)),
    ALGAE_REEF(new LoggedTunableNumber("ArmPivotAlgaeReefRads", 0)),
    BARGE_SCORE(new LoggedTunableNumber("ArmPivotBargeScoreRads", 0)),
    L2_L3(new LoggedTunableNumber("ArmPivotL2andL3ScoreRads", 0)),
    L4(new LoggedTunableNumber("ArmPivotL4ScoreRads", 0)),
    BACK_L2_L2(new LoggedTunableNumber("ArmPivotBackL2andL3ScoreRads", 0)),
    BACK_L4(new LoggedTunableNumber("ArmPivotBackL4ScoreRads", 0)),
    LOLIPOP(new LoggedTunableNumber("ArmPivotLolipopScoreRads", 0)),
    MOVING(new LoggedTunableNumber("ArmPivotMovingRads", -1)),
    OPERATOR_CONTROL(new LoggedTunableNumber("ArmPivotOperatorControlRads", 0));

    private final LoggedTunableNumber rads;

    ArmPivotState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private ArmPivotState goalState;

  private final Alert armPivotMotorDisconnected;

  private final Notification armPivotMotorDisconnectedNotification;

  public ArmPivot(ArmPivotIO io) {
    this.io = io;
    this.goalState = ArmPivotState.ARM_UP;
    this.armPivotMotorDisconnected =
        new Alert("Arm Pivot Motor Disconnected! ", Alert.AlertType.kWarning);
    this.armPivotMotorDisconnectedNotification =
        new Notification(
            NotificationLevel.WARNING, "Arm Pivot Warning, ", "Arm Pivot Motor Disconnected");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ArmPivot", inputs);

    Logger.recordOutput("ArmPivot/GoalState", goalState.toString());
    Logger.recordOutput("ArmPivot/TargetRads", goalState.rads.getAsDouble());

    armPivotMotorDisconnected.set(!inputs.armPivotMotorConnected);

    // if (!inputs.armPivotMotorConnected) {
    //   Elastic.sendNotification(armPivotMotorDisconnectedNotification);
    // }
  }

  public double getArmPivotPositionRads() {
    return inputs.armPivotPositionRads / Constants.MOTOR_TO_ARM_PIVOT_REDUCTION;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getArmPivotPositionRads() - this.goalState.rads.getAsDouble())
        < armPivotTolerance.getAsDouble();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setGoalState(ArmPivotState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case OPERATOR_CONTROL:
        RobotState.getInstance().setArmPivotState(ArmPivotState.OPERATOR_CONTROL);
        break;
      case MOVING:
        DriverStation.reportError("Invalid goal ArmPivotState!", null);
        break;
      default:
        RobotState.getInstance().setArmPivotState(ArmPivotState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
