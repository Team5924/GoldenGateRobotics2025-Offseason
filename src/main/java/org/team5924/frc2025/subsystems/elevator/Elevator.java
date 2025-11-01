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
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import lombok.val;

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.Robot;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.drive.Drive;
import org.team5924.frc2025.util.Elastic.Notification;
import org.team5924.frc2025.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  // Tolerance for position control (in meters)
  private static final LoggedTunableNumber POSITION_TOLERANCE =
      new LoggedTunableNumber("Elevator/PositionTolerance", 0.02);

  /** Creates a new elevator. */
  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public final SysIdRoutine upSysId;
  public final SysIdRoutine downSysId;

  public enum ElevatorState {
    Down(new LoggedTunableNumber("Elevator/", 0.0)),
    PreHandoff(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(36.0))),
    Handoff(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(33.25))),
    SourceIntake(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(53.0))),
    PreScore(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(20.0))),
    Trough(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(38.0))),
    L2(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(15.0))),
    L3(new LoggedTunableNumber("Elevator/", L2.getRawExtension() + Units.inchesToMeters(15.8701))),
    L4(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(54.5 - 0.125))),
    Barge(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(55.0 - 0.125))),
    ScoreL4(new LoggedTunableNumber("Elevator/", L4.getRawExtension() - Units.inchesToMeters(1.0))),
    ScoreL3(new LoggedTunableNumber("Elevator/", L3.getRawExtension() - Units.inchesToMeters(3.5))),
    ScoreL2(new LoggedTunableNumber("Elevator/", L2.getRawExtension() - Units.inchesToMeters(3.5))),
    PostL3(new LoggedTunableNumber("Elevator/", L2.getRawExtension() - Units.inchesToMeters(6.0))), //TODO: Tune
    PostL2(new LoggedTunableNumber("Elevator/", L2.getRawExtension() - Units.inchesToMeters(3.5))), //TODO: Tune
    AutoAlgae(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(21.75))),
    LowAlgae(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(22.25))),
    HighAlgae(new LoggedTunableNumber("Elevator/", LowAlgae.getRawExtension() + Units.inchesToMeters(15.8701))),
    Processor(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(20.0))),
    AlgaeRest(new LoggedTunableNumber("Elevator/", Units.inchesToMeters(15.0))),
    GroundAlgaeIntake(new LoggedTunableNumber("Elevator/", 0.14)),
    PopsiclePickup(new LoggedTunableNumber("Elevator/", 0.065));

    @Getter private final LoggedTunableNumber heightMeters;

    ElevatorState(LoggedTunableNumber heightMeters) {
      this.heightMeters = heightMeters;
    }

    private double getRawExtension() {
      return heightMeters.getAsDouble();
    }
  }

  

  Translation2d endOfManipulatorPose() {
      return new Translation2d(Constants.CORAL_CENTER_OFFSET, 0.0); // TODO: uncomment below once vision is pushed
          // .rotateBy(RobotState.getInstance().getEstimatedPose().getRotation());
          // .plus(RobotState.getInstance().getEstimatedPoseBack().getTranslation());
  }

  public enum AlgaeHeight {
    High(Units.inchesToMeters(15.8701)),
    Low(0.0);
    
    private final double offset;
    AlgaeHeight(double offset) {
      this.offset = offset;
    }
  }

  public AlgaeHeight preferredAlgaeHeight() {
    // Find direction of vector from center of reef to the center of the robot
    // (This is counterclockwise from straight-up)
    Translation2d reefCenter = RobotState.getInstance().isRedAlliance() ? Constants.Reef.redCenter : Constants.Reef.blueCenter;
    double degreesAroundReefCenter = endOfManipulatorPose().minus(reefCenter).getAngle().getDegrees();
    // Mirror angle for red alliance since driver stations always face a high algae,
    // but all our math is from blue alliance perspective so need to invert.
    if (RobotState.getInstance().isRedAlliance())
        degreesAroundReefCenter += 180.0;
    //  Subtract 30 degrees so that 0 aligns with a corner of the reef.
    double algaeDirection = Math.toDegrees(Math.toRadians(degreesAroundReefCenter - 30.0));
    return  (300.0 < algaeDirection && algaeDirection < 360.0
        || 180.0 < algaeDirection && algaeDirection < 240.0
        || 60.0 < algaeDirection && algaeDirection < 120.0) ? AlgaeHeight.Low : AlgaeHeight.High;
  }

  @Getter private ElevatorState goalState;

  private final Alert leftMotorDisconnected;
  private final Alert rightMotorDisconnected;

  private final Notification leftMotorDisconnectedNotification;
  private final Notification rightMotorDisconnectedNotification;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.goalState = ElevatorState.Down;
    RobotState.getInstance().setElevatorState(this.goalState);
    RobotState.getInstance().setElevatorPositionMeters(getElevatorPositionMeters());
    this.leftMotorDisconnected =
        new Alert("Left elevator motor disconnected!", Alert.AlertType.kWarning);
    this.rightMotorDisconnected =
        new Alert("Right elevator motor disconnected!", Alert.AlertType.kWarning);

    leftMotorDisconnectedNotification =
        new Notification(
            NotificationLevel.WARNING, "Elevator Warning", "Left elevator motor disconnected!");

    rightMotorDisconnectedNotification =
        new Notification(
            NotificationLevel.WARNING, "Elevator Warning", "Right elevator motor disconnected!");

    upSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(.75).per(Seconds),
                Volts.of(1),
                Seconds.of(new LoggedTunableNumber("Elevator/SysIdTime", 10).getAsDouble()),
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

    downSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2).per(Seconds),
                Volts.of(2),
                Seconds.of(new LoggedTunableNumber("Elevator/SysIdTime", 10).getAsDouble()),
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("RobotState/ElevatorState", RobotState.getInstance().getElevatorState());
    Logger.recordOutput("Elevator/GoalState", goalState.toString());
    Logger.recordOutput("Elevator/TargetHeight", goalState.heightMeters);

    leftMotorDisconnected.set(!inputs.leftMotorConnected);
    rightMotorDisconnected.set(!inputs.rightMotorConnected);

    // if (!inputs.leftMotorConnected) Elastic.sendNotification(leftMotorDisconnectedNotification);
    // if (!inputs.rightMotorConnected)
    // Elastic.sendNotification(rightMotorDisconnectedNotification);

    RobotState.getInstance().setElevatorPositionMeters(getElevatorPositionMeters());

    io.periodicUpdates();
  }

  private double getElevatorPositionMeters() {
    return Radians.of(inputs.leftPositionRads).in(Rotations)
        * 2
        * Math.PI
        * Constants.SPROCKET_RADIUS.in(Meters)
        / Constants.MOTOR_TO_ELEVATOR_REDUCTION;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getElevatorPositionMeters() - this.goalState.heightMeters.getAsDouble())
        < POSITION_TOLERANCE.getAsDouble();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setSoftStopOn() {
    io.setSoftStopOn();
  }

  public void setSoftStopOff() {
    io.setSoftStopOff();
  }

  // public void setGoalState(ElevatorState goalState) {
  //   this.goalState = goalState;
  //   switch (goalState) {
  //     case MANUAL -> RobotState.getInstance().setElevatorState(ElevatorState.MANUAL);
  //     case MOVING ->
  //         DriverStation.reportError(
  //             "MOVING is an intermediate state and cannot be set as a goal state!", null);
  //     default -> {
  //       RobotState.getInstance().setElevatorState(goalState);
  //       io.setHeight(goalState.heightMeters.getAsDouble());
  //     }
  //   }
  // }
}
