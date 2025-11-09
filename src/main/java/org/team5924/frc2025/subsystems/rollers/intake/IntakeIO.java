/*
 * IntakeIO.java
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

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorConnected = true;
    public double intakePositionRads = 0.0;
    public double intakeVelocityRadsPerSec = 0.0;
    public double intakeAppliedVoltage = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeTorqueCurrentAmps = 0.0;
    public double intakeTempCelsius = 0.0;

    public boolean alignerMotorConnected = true;
    public double alignerPositionRads = 0.0;
    public double alignerVelocityRadsPerSec = 0.0;
    public double alignerAppliedVoltage = 0.0;
    public double alignerSupplyCurrentAmps = 0.0;
    public double alignerTorqueCurrentAmps = 0.0;
    public double alignerTempCelsius = 0.0;

    // public boolean beamBreakConnected = true;
    // public boolean beamBreakUnbroken = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  default void runVolts(double intakeVolts, double alignerVolts) {}
}
