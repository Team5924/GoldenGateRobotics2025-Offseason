package org.team5924.frc2025.subsystems.pivot;

import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ArmPivotIOKrakenFOC implements ArmPivotIO{
    private static TalonFX armPivotKraken;
    private static CANcoder armPivotCANcoder;

    private final StatusSignal<Angle> armPivotCANcoderAbsolutePositionRotations;
    private final StatusSignal<Angle> armPivotCANcoderRelativePositionRotations;
    private final StatusSignal<Angle> armPivotPosition;
    private final StatusSignal<AngularVelocity> armPivotVelocity;
    private final StatusSignal<Voltage> armPivotAppliedVolts;
    private final StatusSignal<Current> armPivotSupplyCurrent;
    private final StatusSignal<Current> armPivotTorqueCurrent;
    private final StatusSignal<Temperature> armPivotTempCelsius;

    private final VoltageOut voltageControl =
      new VoltageOut(0).withUpdateFreqHz(0.0).withEnableFOC(true);
    private final PositionVoltage positionControl =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

    private final LoggedTunableNumber armPivotMotorkP = new LoggedTunableNumber("ArmPivotMotorkP", 0);
    private final LoggedTunableNumber armPivotMotorkI = new LoggedTunableNumber("ArmPivotMotorkI", 0);
    private final LoggedTunableNumber armPivotMotorkD = new LoggedTunableNumber("ArmPivotMotorkD", 0);
    private final LoggedTunableNumber armPivotMotorkS = new LoggedTunableNumber("ArmPivotMotorkS", 0);
    private final LoggedTunableNumber armPivotCANcoderMagnetOffsetRads =
      new LoggedTunableNumber("ArmPivotCANcoderOffsetRads", 0);
    private final LoggedTunableNumber armPivotSensorDiscontinuityPoint =
      new LoggedTunableNumber("ArmPivotSensorDiscontinuityPoint", .5);


    public ArmPivotIOKrakenFOC(){
        armPivotKraken = new TalonFX(Constants.ARM_PIVOT_CAN_ID);
        armPivotCANcoder = new CANcoder(Constants.ARM_PIVOT_CANCODER_ID);

        CANcoderConfiguration armPivotCANcoderConfiguration = new CANcoderConfiguration();
        armPivotCANcoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(
          armPivotSensorDiscontinuityPoint.getAsDouble());
        armPivotCANcoderConfiguration.MagnetSensor.SensorDirection = 
          SensorDirectionValue.Clockwise_Positive;
        armPivotCANcoderConfiguration.MagnetSensor.MagnetOffset = 
          Units.radiansToRotations(armPivotCANcoderMagnetOffsetRads.getAsDouble());
        armPivotCANcoder.getConfigurator().apply(armPivotCANcoderConfiguration, 1);

        //Motor General Config

        //TODO: Change These If Incorrect
        final TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
        krakenConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        krakenConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        krakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        krakenConfig.Feedback.SensorToMechanismRatio = Constants.MOTOR_TO_ARM_PIVOT_REDUCTION;

        final Slot0Configs controllerConfig = new Slot0Configs();
        controllerConfig.kP = armPivotMotorkP.getAsDouble();
        controllerConfig.kI = armPivotMotorkI.getAsDouble();
        controllerConfig.kD = armPivotMotorkD.getAsDouble();
        controllerConfig.kS = armPivotMotorkS.getAsDouble();
        armPivotKraken.getConfigurator().apply(krakenConfig);
        armPivotKraken.getConfigurator().apply(controllerConfig);

        armPivotCANcoderAbsolutePositionRotations = armPivotCANcoder.getAbsolutePosition();
        armPivotCANcoderRelativePositionRotations = armPivotCANcoder.getPosition();

        armPivotPosition = armPivotKraken.getPosition();
        armPivotVelocity = armPivotKraken.getVelocity();
        armPivotAppliedVolts = armPivotKraken.getMotorVoltage();
        armPivotSupplyCurrent = armPivotKraken.getSupplyCurrent();
        armPivotTorqueCurrent = armPivotKraken.getTorqueCurrent();
        armPivotTempCelsius = armPivotKraken.getDeviceTemp();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            armPivotPosition,
            armPivotVelocity,
            armPivotAppliedVolts,
            armPivotSupplyCurrent,
            armPivotTorqueCurrent,
            armPivotTempCelsius);

        BaseStatusSignal.setUpdateFrequencyForAll(
            500,
            armPivotCANcoderAbsolutePositionRotations,
            armPivotCANcoderRelativePositionRotations);
    }

    @Override
  public void updateInputs(ArmPivotIOInputs inputs) {
    inputs.armPivotMotorConnected =
        BaseStatusSignal.refreshAll(
                armPivotPosition,
                armPivotVelocity,
                armPivotAppliedVolts,
                armPivotSupplyCurrent,
                armPivotTorqueCurrent,
                armPivotTempCelsius)
            .isOK();

    inputs.armPivotPositionRads = armPivotPosition.getValue().in(Radians);
    inputs.armPivotVelocityRadsPerSec = armPivotVelocity.getValue().in(RadiansPerSecond);
    inputs.armPivotAppliedVolts = armPivotAppliedVolts.getValue().in(Volts);
    inputs.armPivotSupplyCurrentAmps = armPivotSupplyCurrent.getValue().in(Amps);
    inputs.armPivotTorqueCurrentAmps = armPivotTorqueCurrent.getValue().in(Amps);
    inputs.armPivotTempCelsius = armPivotTempCelsius.getValue().in(Celsius);
    inputs.armPivotCANcoderAbsolutePositionRads =
        Units.rotationsToRadians(armPivotCANcoderAbsolutePositionRotations.getValueAsDouble())
            - armPivotCANcoderMagnetOffsetRads.getAsDouble();
    inputs.armPivotCANcoderRelativePositionRads =
        Units.rotationsToRadians(armPivotCANcoderRelativePositionRotations.getValueAsDouble())
            - armPivotCANcoderMagnetOffsetRads.getAsDouble();
  }

  @Override
  public void setVoltage(double volts){
    armPivotKraken.setControl(voltageControl.withOutput(volts));
  }
  @Override
  public void setPosition(double rads){
    armPivotKraken.setControl(positionControl.withPosition(rads));
  }
}
