/*
 * IntakePivotIOKrakenFOC.java
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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class IntakePivotIOKrakenFOC implements IntakePivotIO {
  private final TalonFX intakePivotKraken;
  // private final CANcoder intakePivotCANcoder;

  // Motor Status Signals
  private final StatusSignal<Angle> intakePivotPosition;
  private final StatusSignal<AngularVelocity> intakePivotVelocity;
  private final StatusSignal<Voltage> intakePivotAppliedVolts;
  private final StatusSignal<Current> intakePivotSupplyCurrent;
  private final StatusSignal<Current> intakePivotTorqueCurrent;
  private final StatusSignal<Temperature> intakePivotTempCelsius;

  // PID for the Motor
  LoggedTunableNumber intakePivotMotorkP = new LoggedTunableNumber("IntakePivotMotorkP", 0);
  LoggedTunableNumber intakePivotMotorkI = new LoggedTunableNumber("IntakePivotMotorkI", 0);
  LoggedTunableNumber intakePivotMotorkD = new LoggedTunableNumber("IntakePivotMotorkD", 0);
  LoggedTunableNumber intakePivotMotorkS = new LoggedTunableNumber("IntakePivotMotorkS", 0);

  // Refresh on what this does, cuz im lowk dumb rn
  private final VoltageOut voltageControl =
      new VoltageOut(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final PositionVoltage positionControl =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  /*  for the CANcoder
  LoggedTunableNumber intakePivotCANcoderMagnetOffsetRads =
    new LoggedTunableNumber("AlgaePivotCANcoderOffsetRads", 0);
  LoggedTunableNumber intakePivotSensorDiscontinuityPoint =
    new LoggedTunableNumber("AlgaePivotSensorDiscontinuityPoint", .5);
  */
  public IntakePivotIOKrakenFOC() {
    intakePivotKraken = new TalonFX(Constants.INTAKE_PIVOT_CAN_ID, Constants.INTAKE_PIVOT_OUT_BUS);

    // Config
    final TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    krakenConfig.CurrentLimits.SupplyCurrentLimit = Constants.INTAKE_PIVOT_CURRENT_LIMIT;
    krakenConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    krakenConfig.MotorOutput.Inverted = Constants.INTAKE_PIVOT_INVERT;
    krakenConfig.MotorOutput.NeutralMode = Constants.INTAKE_PIVOT_BRAKE;
    krakenConfig.Feedback.SensorToMechanismRatio = Constants.MOTOR_TO_INTAKE_PIVOT_REDUCTION;

    // Kraken PID Slot Configs

    final Slot0Configs controllerConfigs = new Slot0Configs();
    controllerConfigs.kP = intakePivotMotorkP.getAsDouble();
    controllerConfigs.kI = intakePivotMotorkI.getAsDouble();
    controllerConfigs.kD = intakePivotMotorkD.getAsDouble();
    controllerConfigs.kS = intakePivotMotorkS.getAsDouble();

    intakePivotPosition = intakePivotKraken.getPosition();
    intakePivotVelocity = intakePivotKraken.getVelocity();
    intakePivotAppliedVolts = intakePivotKraken.getMotorVoltage();
    intakePivotSupplyCurrent = intakePivotKraken.getSupplyCurrent();
    intakePivotTorqueCurrent = intakePivotKraken.getTorqueCurrent();
    intakePivotTempCelsius = intakePivotKraken.getDeviceTemp();

    intakePivotKraken.getConfigurator().apply(krakenConfig, 1.0);
    intakePivotKraken.getConfigurator().apply(controllerConfigs, 1.0);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs input) {
    input.intakePivotMotorConnected =
        BaseStatusSignal.refreshAll(
                intakePivotPosition,
                intakePivotVelocity,
                intakePivotAppliedVolts,
                intakePivotSupplyCurrent,
                intakePivotTorqueCurrent,
                intakePivotTempCelsius)
            .isOK();

    input.intakePivotPositionRads = intakePivotPosition.getValue().in(Radians);
    input.intakePivotVelocityRadsPerSec = intakePivotVelocity.getValue().in(RadiansPerSecond);
    input.intakePivotAppliedVolts = intakePivotAppliedVolts.getValue().in(Volts);
    input.intakePivotSupplyCurrentAmps = intakePivotSupplyCurrent.getValue().in(Amps);
    input.intakePivotTorqueCurrentAmps = intakePivotTorqueCurrent.getValue().in(Amps);
    input.intakePivotTempCelsius = intakePivotTempCelsius.getValue().in(Celsius);
  }

  @Override
  public void setVoltage(double volts) {
    intakePivotKraken.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    intakePivotKraken.setControl(positionControl.withPosition(rads));
  }

  @Override
  public void setSoftStopOn() {}

  @Override
  public void setSoftStopOff() {}

  /** Stop pivot */
  @Override
  public void stop() {}
}
