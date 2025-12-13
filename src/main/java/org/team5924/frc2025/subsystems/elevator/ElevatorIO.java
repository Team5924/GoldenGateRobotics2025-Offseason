/*
 * ElevatorIO.java
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

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {

    /* Motor Data */
    public boolean motorConnected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    /* Real Elevator Data */
    public double realPos = 0.0;
    public double realVel = 0.0;
    public double realAccl = 0.0;

    /* Motion Target Elevator Data */
    public double targetPos = 0.0;
    public double targetVel = 0.0;

    public double setpointMeters = 0.0;
  }


  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Runs updates periodically for logged tunable numbers, alerts
   */
  public default void periodicUpdates() {}

  /**
   * Sets te height of the elevator using MotionMagic
   * 
   * @param heightMeters the height at which to set the robot
   */
  public default void setHeight(double heightMeters) {}

  /**
   * Runs the elevator at the specified voltage
   *
   * @param volts Voltage to apply
   */
  public default void setVoltage(double volts) {}

  /**
   * enables the soft stop
   */
  public default void setSoftStopOn() {}

  /**
   * disables the soft stop
   */
  public default void setSoftStopOff() {}
}
