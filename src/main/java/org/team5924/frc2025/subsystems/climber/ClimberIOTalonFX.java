/*
 * ClimberIOTalonFX.java
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

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climbTalon;
  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVoltage;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Current> climbTorqueCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  // private final CANcoder cancoder;
  // private final StatusSignal<Angle> cancoderPosition;
  // private final StatusSignal<Voltage> cancoderSupplyVoltage;

  private final TalonFX grabTalon;
  private final StatusSignal<Angle> grabPosition;
  private final StatusSignal<AngularVelocity> grabVelocity;
  private final StatusSignal<Voltage> grabAppliedVoltage;
  private final StatusSignal<Current> grabSupplyCurrent;
  private final StatusSignal<Current> grabTorqueCurrent;
  private final StatusSignal<Temperature> grabTempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  private final double climbReduction;

  public ClimberIOTalonFX() {
    // Climb motor
    {
      climbReduction = Constants.Climber.REDUCTION;
      climbTalon = new TalonFX(Constants.Climber.CAN_ID, Constants.Climber.BUS);
      climbTalon.getConfigurator().apply(Constants.Climber.CONFIG);

      // Get select status signals and set update frequency
      climbPosition = climbTalon.getPosition();
      climbVelocity = climbTalon.getVelocity();
      climbAppliedVoltage = climbTalon.getMotorVoltage();
      climbSupplyCurrent = climbTalon.getSupplyCurrent();
      climbTorqueCurrent = climbTalon.getTorqueCurrent();
      climbTempCelsius = climbTalon.getDeviceTemp();

      BaseStatusSignal.setUpdateFrequencyForAll(
          50.0,
          climbPosition,
          climbVelocity,
          climbAppliedVoltage,
          climbSupplyCurrent,
          climbTorqueCurrent,
          climbTempCelsius);

      climbTalon.setPosition(0);
    }

    // CANcoder
    {
      // cancoder = new CANcoder(Constants.Climber.CANCODER_ID, Constants.Climber.BUS);

      // // Configure
      // MagnetSensorConfigs config = new MagnetSensorConfigs();
      // config.MagnetOffset = Constants.Climber.CANCODER_MAGNET_OFFSET;
      // config.SensorDirection = Constants.Climber.CANCODER_SENSOR_DIRECTION;
      // config.AbsoluteSensorDiscontinuityPoint =
      //     Constants.Climber.CANCODER_SENSOR_DISCONTINUITY_POINT;
      // cancoder.getConfigurator().apply(config);

      // cancoderPosition = cancoder.getAbsolutePosition();
      // cancoderSupplyVoltage = cancoder.getSupplyVoltage();

      // BaseStatusSignal.setUpdateFrequencyForAll(50, cancoderPosition, cancoderSupplyVoltage);
    }

    // Grab motor
    {
      grabTalon = new TalonFX(Constants.Grabber.CAN_ID, Constants.Climber.BUS);
      grabTalon.getConfigurator().apply(Constants.Grabber.CONFIG);

      // Get select status signals and set update frequency
      grabPosition = grabTalon.getPosition();
      grabVelocity = grabTalon.getVelocity();
      grabAppliedVoltage = grabTalon.getMotorVoltage();
      grabSupplyCurrent = grabTalon.getSupplyCurrent();
      grabTorqueCurrent = grabTalon.getTorqueCurrent();
      grabTempCelsius = grabTalon.getDeviceTemp();

      BaseStatusSignal.setUpdateFrequencyForAll(
          50.0,
          grabPosition,
          grabVelocity,
          grabAppliedVoltage,
          grabSupplyCurrent,
          grabTorqueCurrent,
          grabTempCelsius);

      // Disables status signals not called for update above
      // grabTalon.setPosition(0);
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    {
      inputs.climbMotorConnected =
          BaseStatusSignal.refreshAll(
                  climbPosition,
                  climbVelocity,
                  climbAppliedVoltage,
                  climbSupplyCurrent,
                  climbTorqueCurrent,
                  climbTempCelsius)
              .isOK();
      inputs.climbPositionRads =
          Units.rotationsToRadians(climbPosition.getValueAsDouble()) / climbReduction;
      inputs.climbVelocityRadsPerSec =
          Units.rotationsToRadians(climbVelocity.getValueAsDouble()) / climbReduction;
      inputs.climbAppliedVoltage = climbAppliedVoltage.getValueAsDouble();
      inputs.climbSupplyCurrentAmps = climbSupplyCurrent.getValueAsDouble();
      inputs.climbTorqueCurrentAmps = climbTorqueCurrent.getValueAsDouble();
      inputs.climbTempCelsius = climbTempCelsius.getValueAsDouble();
    }

    {
      // inputs.cancoderConnected =
      //     BaseStatusSignal.refreshAll(cancoderPosition, cancoderSupplyVoltage).isOK();
      // inputs.cancoderPosition = Units.rotationsToRadians(cancoderPosition.getValueAsDouble());
      // inputs.cancoderSupplyVoltage = cancoderSupplyVoltage.getValueAsDouble();
    }

    {
      inputs.grabMotorConnected =
          BaseStatusSignal.refreshAll(
                  grabPosition,
                  grabVelocity,
                  grabAppliedVoltage,
                  grabSupplyCurrent,
                  grabTorqueCurrent,
                  grabTempCelsius)
              .isOK();
      inputs.grabPositionRads = Units.rotationsToRadians(grabPosition.getValueAsDouble());
      inputs.grabVelocityRadsPerSec = Units.rotationsToRadians(grabVelocity.getValueAsDouble());
      inputs.grabAppliedVoltage = grabAppliedVoltage.getValueAsDouble();
      inputs.grabSupplyCurrentAmps = grabSupplyCurrent.getValueAsDouble();
      inputs.grabTorqueCurrentAmps = grabTorqueCurrent.getValueAsDouble();
      inputs.grabTempCelsius = grabTempCelsius.getValueAsDouble();
    }
  }

  @Override
  public void runClimbVolts(double volts) {
    if (atSoftStop(volts)) return; // TODO: test that this function works as intended
    climbTalon.setControl(voltageOut.withOutput(volts));
  }

  public boolean atSoftStop(double volts) {
    double currentAngle =
        Units.rotationsToRadians(climbPosition.getValueAsDouble()) / climbReduction;
    return (currentAngle >= Constants.Climber.MAX_RADS
            && volts < 0) // motor moving in positive rads, stop at upper bound
        || (currentAngle <= Constants.Climber.MIN_RADS
            && volts > 0); // motor moving in negative rads, stop at lower bound
  }

  @Override
  public void setClimbControl(CoastOut control) {
    climbTalon.setControl(control);
  }

  @Override
  public void runGrabVolts(double volts) {
    grabTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setClimbAngle(double rads) throws IllegalArgumentException {
    if (rads < Constants.Climber.MIN_RADS || rads > Constants.Climber.MAX_RADS) {
      String message =
          "Cannot set climber angle to "
              + rads
              + " rads.  This value extends past the climber angle boundary.";
      Logger.recordOutput("Climber/InvalidAngle", message);
      throw new IllegalArgumentException(message);
    }
    rads = Math.max(Constants.Climber.MIN_RADS, Math.min(rads, Constants.Climber.MAX_RADS));
    climbTalon.setControl(positionOut.withPosition(Radians.of(rads)));
  }

  @Override
  public void disableClimbTalon() {
    climbTalon.disable();
  }

  @Override
  public void disableGrabTalon() {
    grabTalon.disable();
  }
}
