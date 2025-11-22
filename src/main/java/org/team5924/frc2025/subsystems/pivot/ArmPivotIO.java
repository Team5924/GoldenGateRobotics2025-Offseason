/*
 * ArmPivotIO.java
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

public interface ArmPivotIO {

  @AutoLog
  public class ArmPivotIOInputs {
    public boolean armPivotMotorConnected = true;

    public double armPivotPositionRads = 0;
    public double armPivotVelocityRadsPerSec = 0;
    public double armPivotAppliedVolts = 0;
    public double armPivotSupplyCurrentAmps = 0;
    public double armPivotTorqueCurrentAmps = 0;
    public double armPivotTempCelsius = 0;

    public boolean armPivotCANcoderConnected = true;
    public double armPivotCANcoderRelativePositionRads = 0;
    public double armPivotCANcoderAbsolutePositionRads = 0;
  }

  public default void updateInputs(ArmPivotIOInputs input) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double rads) {}

  public default void stop() {}
}
