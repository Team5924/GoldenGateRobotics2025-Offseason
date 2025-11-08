/*
 * ClimberIO.java
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
import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.Constants;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean climbMotorConnected = true;
    public double climbPositionRads = 0.0;
    public double climbVelocityRadsPerSec = 0.0;
    public double climbAppliedVoltage = 0.0;
    public double climbSupplyCurrentAmps = 0.0;
    public double climbTorqueCurrentAmps = 0.0;
    public double climbTempCelsius = 0.0;

    public boolean grabMotorConnected = true;
    public double grabPositionRads = 0.0;
    public double grabVelocityRadsPerSec = 0.0;
    public double grabAppliedVoltage = 0.0;
    public double grabSupplyCurrentAmps = 0.0;
    public double grabTorqueCurrentAmps = 0.0;
    public double grabTempCelsius = 0.0;
  }

  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Runs the climb motor at the specified voltage
   *
   * @param volts Voltage to apply
   */
  public default void runClimbVolts(double volts) {}

  public default void setClimbControl(CoastOut control) {}

  /**
   * Runs the grab motor at the specified voltage
   *
   * @param volts Voltage to apply
   */
  public default void runGrabVolts(double volts) {}

  /**
   * Sets the target angle for the climber motor
   *
   * @param rads Target angle in radians, must be between {@link Constants}.CLIMBER_MIN_RADS and
   *     {@link Constants}.CLIMBER_MAX_RADS
   * @throws IllegalArgumentException if value does not fall in range
   */
  public default void setClimbAngle(double rads) {}

  public default void disableClimbTalon() {}

  public default void disableGrabTalon() {}

  /** stops the motor */
  default void stop() {}
}
