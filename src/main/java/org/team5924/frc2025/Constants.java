/*
 * Constants.java
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

package org.team5924.frc2025;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean TUNING_MODE = false;
  public static final boolean ALLOW_ASSERTS = false;
  public static final double SLOW_MODE_MULTI = 0.5;

  /* Field */
  public static final double FIELD_BORDER_MARGIN = 0.5;
  public static final AprilTagFieldLayout field =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final double FIELD_WIDTH = field.getFieldWidth();
  public static final double FIELD_LENGTH = field.getFieldLength();
  public static final double CORAL_STATION_RADIANS_NORMAL = 0.959931;

  /* General */
  public static final double LOOP_PERIODIC_SECONDS = 0.02;
  /* Climber */
  public static final int CLIMBER_CAN_ID = 40;
  public static final String CLIMBER_BUS = "drive";
  public static final int CLIMBER_SUPPLY_CURRENT_LIMIT = 60;
  public static final int CLIMBER_STATOR_CURRENT_LIMIT = 60;
  public static final InvertedValue CLIMBER_INVERT = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue CLIMBER_NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final double CLIMBER_REDUCTION = 60;
  public static final double CLIMBER_MIN_RADS = Math.toRadians(-55);
  public static final double CLIMBER_MAX_RADS = Math.toRadians(100);

  public static final boolean CLIMBER_REQUIRE_AT_GOAL = false;

  /* Climber cancoder */
  // public static final int CLIMBER_CANCODER_ID = 41;
  // public static final double CLIMBER_CANCODER_MAGNET_OFFSET = -0.244384765625;
  // public static final SensorDirectionValue CLIMBER_CANCODER_SENSOR_DIRECTION =
  //     SensorDirectionValue.Clockwise_Positive;
  // public static final double CLIMBER_CANCODER_SENSOR_DISCONTINUITY_POINT = 0.5;

  /* Grabber */
  public static final int GRABBER_CAN_ID = 41;
  public static final int GRABBER_SUPPLY_CURRENT_LIMIT = 35;
  public static final int GRABBER_STATOR_CURRENT_LIMIT = 35;
  public static final InvertedValue GRABBER_INVERT = InvertedValue.Clockwise_Positive;

  /* Intake */
  public static final int INTAKE_CAN_ID = 30;
  public static final String INTAKE_OUT_BUS = "rio";
  public static final int INTAKE_CURRENT_LIMIT = 100;
  public static final boolean INTAKE_INVERT = true;
  public static final boolean INTAKE_BRAKE = true;
  public static final double INTAKE_REDUCTION = 1.0;

  // public static final int INTAKE_BEAM_BREAK_ID = 52;
  // public static final boolean INTAKE_BEAM_BREAK = false;

  /* Aligner */
  public static final int ALIGNER_CAN_ID = 31;
  public static final String ALIGNER_OUT_BUS = "rio";
  public static final int ALIGNER_CURRENT_LIMIT = 60;
  public static final boolean ALIGNER_INVERT = false;
  public static final boolean ALIGNER_BRAKE = true;
  public static final double ALIGNER_REDUCTION = 1.0;

  /* # Intake Pivot # */
  public static final int INTAKE_PIVOT_CAN_ID = 32;
  public static final String INTAKE_PIVOT_OUT_BUS = "rio";
  public static final int INTAKE_PIVOT_CURRENT_LIMIT = 60;
  public static final InvertedValue INTAKE_PIVOT_INVERT = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue INTAKE_PIVOT_BRAKE = NeutralModeValue.Brake;
  public static final double MOTOR_TO_INTAKE_PIVOT_REDUCTION =
      1.0 / ((12.0 / 40.0) * (18.0 / 46.0) * (18.0 / 60.0) * (12.0 / 32.0));

  public static final double INTAKE_PIVOT_MIN_RADS =
      Math.toRadians(-2.0); // TODO: test these values
  public static final double INTAKE_PIVOT_MAX_RADS = Math.toRadians(128.0);

  public static final double INTAKE_PIVOT_JOYSTICK_DEADZONE = 0.05;

  /* Elevator */
  public static final String ELEVATOR_CANDI_BUS = "rio";
  public static final int ELEVATOR_CAN_ID = 50;
  public static final int ELEVATOR_CANDI_ID = 51;
  public static final TalonFXConfiguration ELEVATOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(80).withStatorCurrentLimit(80))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withPeakForwardDutyCycle(1)
                  .withPeakReverseDutyCycle(-1));

  public static final CANdiConfiguration ELEVATOR_CANDI_CONFIGS =
      new CANdiConfiguration()
          .withDigitalInputs(
              new DigitalInputsConfigs()
                  .withS1CloseState(S1CloseStateValue.CloseWhenLow)
                  .withS2CloseState(S2CloseStateValue.CloseWhenLow));

  public static final double MOTOR_TO_ELEVATOR_REDUCTION = 4.00;
  public static final Distance ELEVATOR_SPROCKET_RADIUS = Inches.of(0.6405);
  public static final double ELEVATOR_CANCODER_OFFSET = 0.00;
}
