/*
 * Elevator.java
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

package org.team5924.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public final SysIdRoutine upSysId;
  public final SysIdRoutine downSysId;

  private final LoggedTunableNumber sysIdTime = new LoggedTunableNumber("Elevator/SysIdTime", 10);

  public enum ElevatorState {
    IDLE(new LoggedTunableNumber("Elevator/IdleHeight", 0.0)),
    DOWN(new LoggedTunableNumber("Elevator/DownHeight", 0.0)),
    HANDOFF(new LoggedTunableNumber("Elevator/CoralHandoffHeight", 0.0)),
    LOLIPOP(new LoggedTunableNumber("Elevator/LolipopIntakeHeight", 0.0)),
    L2(new LoggedTunableNumber("Elevator/L2Height", 0.0)),
    L3(new LoggedTunableNumber("Elevator/L3Height", 0.0)),
    L4(new LoggedTunableNumber("Elevator/L4Height", 0.0)),
    SCORE_L2(new LoggedTunableNumber("Elevator/ScoreL2Height", 0.0)),
    SCORE_L3(new LoggedTunableNumber("Elevator/ScoreL3Height", 0.0)),
    SCORE_L4(new LoggedTunableNumber("Elevator/ScoreL4Height", 0.0)),
    ALGAE_LOW(new LoggedTunableNumber("Elevator/AlgaeLowHeight", 0.0)),
    ALGAE_HIGH(new LoggedTunableNumber("Elevator/AlgaeHighHeight", 0.0)),
    ALGAE_GROUND(new LoggedTunableNumber("Elevator/AlgaeGroundIntakeHeight", 0.0)),
    BARGE(new LoggedTunableNumber("Elevator/ScoreBargeHeight", 0.0)),
    PROCESSOR(new LoggedTunableNumber("Elevator/ScoreProcessorHeight", 0.0)),
    MANUAL(new LoggedTunableNumber("Elevator/ManualVoltage", 0.0)),
    MOVING(new LoggedTunableNumber("Elevator/Moving", 0.0));

    private final LoggedTunableNumber heightMeters;

    ElevatorState(LoggedTunableNumber heightMeters) {
      this.heightMeters = heightMeters;
    }
  }

  @Getter private ElevatorState goalState;

  private final Alert elevatorMotorDisconnected;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.goalState = ElevatorState.IDLE;
    RobotState.getInstance().setElevatorState(this.goalState);
    elevatorMotorDisconnected = new Alert("Elevator Motor Disconnected!", Alert.AlertType.kWarning);

    upSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(.75).per(Seconds),
                Volts.of(1),
                Seconds.of(sysIdTime.getAsDouble()),
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

    downSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2).per(Seconds),
                Volts.of(2),
                Seconds.of(sysIdTime.getAsDouble()),
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/Height", goalState.heightMeters.get());
    Logger.recordOutput("Elevator/GoalState", goalState.toString());
    Logger.recordOutput("Elevator/ElevatorState", RobotState.getInstance().getElevatorState());

    elevatorMotorDisconnected.set(!inputs.motorConnected);

    RobotState.getInstance().setElevatorPositionMeters(getElevatorPositionMeters());

    io.periodicUpdates();
  }

  private double getElevatorPositionMeters() {
    return inputs.positionRads
        * Constants.ELEVATOR_SPROCKET_RADIUS.in(Meters)
        / Constants.MOTOR_TO_ELEVATOR_REDUCTION;
  }

  public void setGoalState(ElevatorState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case MANUAL:
        RobotState.getInstance().setElevatorState(ElevatorState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "MOVING is an intermediate state and cannot be set as a goal state!", null);
        break;
      default:
        RobotState.getInstance().setElevatorState(goalState);
        io.setHeight(goalState.heightMeters.get());
        break;
    }
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}
