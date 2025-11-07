/*
 * IntakePivotIO.java
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

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public boolean intakePivotMotorConnected = true;
    public double intakePivotPositionRads = 0.0;
    public double intakePivotVelocityRadsPerSec = 0.0;
    public double intakePivotAppliedVolts = 0.0;
    public double intakePivotSupplyCurrentAmps = 0.0;
    public double intakePivotTorqueCurrentAmps = 0.0;
    public double intakePivotTempCelsius = 0.0;

    /* If CANcoder installed

    public boolean intakePivotCANcoderConnected = true;
    public double intakePivotCANcoderAbsolutePositionRads = 0.0;
    public double intakePivotCANcoderRelativePositionRads = 0.0;
    */
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  /** Run pivot at volts */
  public default void setVoltage(double volts) {}

  public default void setPosition(double rads) {}

  public default void setSoftStopOn() {}

  public default void setSoftStopOff() {}

  /** Stop pivot */
  public default void stop() {}
}
