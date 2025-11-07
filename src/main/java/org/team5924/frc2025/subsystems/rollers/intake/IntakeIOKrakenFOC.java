package org.team5924.frc2025.subsystems.rollers.intake;

import org.team5924.frc2025.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;


public class IntakeIOKrakenFOC implements IntakeIO {
    private final TalonFX intakeTalon;
    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeAppliedVoltage;
    private final StatusSignal<Current> intakeSupplyCurrent;
    private final StatusSignal<Current> intakeTorqueCurrent;
    private final StatusSignal<Temperature> intakeTempCelsius;

    private final TalonFX alignerTalon;
    private final StatusSignal<Angle> alignerPosition;
    private final StatusSignal<AngularVelocity> alignerVelocity;
    private final StatusSignal<Voltage> alignerAppliedVoltage;
    private final StatusSignal<Current> alignerSupplyCurrent;
    private final StatusSignal<Current> alignerTorqueCurrent;
    private final StatusSignal<Temperature> alignerTempCelsius;

    private final DigitalInput beamBreakSensor;

    private static final int intakeId = Constants.INTAKE_CAN_ID;
    private static final int alignerId = Constants.ALIGNER_CAN_ID;
    private static final String bus = Constants.INTAKE_OUT_BUS;
    private static final int currentLimitAmps = Constants.INTAKE_CURRENT_LIMIT;
    private static final boolean invert = Constants.INTAKE_INVERT;
    private static final boolean intakeBrake = Constants.INTAKE_BRAKE;
    private static final boolean alignerBrake = Constants.ALIGNER_BRAKE;
    private static final double reduction = Constants.INTAKE_REDUCTION;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

    public IntakeIOKrakenFOC() {
        // intake motor
        {
            intakeTalon = new TalonFX(intakeId, bus);

            // Configure TalonFX
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.Inverted =
                invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = intakeBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            intakeTalon.getConfigurator().apply(config);

            // Get select status signals and set update frequency
            intakePosition = intakeTalon.getPosition();
            intakeVelocity = intakeTalon.getVelocity();
            intakeAppliedVoltage = intakeTalon.getMotorVoltage();
            intakeSupplyCurrent = intakeTalon.getSupplyCurrent();
            intakeTorqueCurrent = intakeTalon.getTorqueCurrent();
            intakeTempCelsius = intakeTalon.getDeviceTemp();
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                intakePosition,
                intakeVelocity,
                intakeAppliedVoltage,
                intakeSupplyCurrent,
                intakeTorqueCurrent,
                intakeTempCelsius);

            // Disables status signals not called for update above
            intakeTalon.optimizeBusUtilization(0, 1.0);
        }

        // aligner motor
        {
            alignerTalon = new TalonFX(alignerId, bus);

            // Configure TalonFX
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.Inverted =
                invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = alignerBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            alignerTalon.getConfigurator().apply(config);

            // Get select status signals and set update frequency
            alignerPosition = alignerTalon.getPosition();
            alignerVelocity = alignerTalon.getVelocity();
            alignerAppliedVoltage = alignerTalon.getMotorVoltage();
            alignerSupplyCurrent = alignerTalon.getSupplyCurrent();
            alignerTorqueCurrent = alignerTalon.getTorqueCurrent();
            alignerTempCelsius = alignerTalon.getDeviceTemp();
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                alignerPosition,
                alignerVelocity,
                alignerAppliedVoltage,
                alignerSupplyCurrent,
                alignerTorqueCurrent,
                alignerTempCelsius);

            // Disables status signals not called for update above
            alignerTalon.optimizeBusUtilization(0, 1.0);
        }
    
        {
            beamBreakSensor = new DigitalInput(Constants.INTAKE_BEAM_BREAK_ID);
        }
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected =
            BaseStatusSignal.refreshAll(
                    intakePosition, intakeVelocity, intakeAppliedVoltage, intakeSupplyCurrent, intakeTorqueCurrent, intakeTempCelsius)
                .isOK();
        inputs.intakePositionRads = Units.rotationsToRadians(intakePosition.getValueAsDouble()) / reduction;
        inputs.intakeVelocityRadsPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble()) / reduction;
        inputs.intakeAppliedVoltage = intakeAppliedVoltage.getValueAsDouble();
        inputs.intakeSupplyCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
        inputs.intakeTorqueCurrentAmps = intakeTorqueCurrent.getValueAsDouble();
        inputs.intakeTempCelsius = intakeTempCelsius.getValueAsDouble();

        inputs.alignerMotorConnected =
            BaseStatusSignal.refreshAll(
                    alignerPosition, alignerVelocity, alignerAppliedVoltage, alignerSupplyCurrent, alignerTorqueCurrent, alignerTempCelsius)
                .isOK();
        inputs.alignerPositionRads = Units.rotationsToRadians(alignerPosition.getValueAsDouble()) / reduction;
        inputs.alignerVelocityRadsPerSec = Units.rotationsToRadians(alignerVelocity.getValueAsDouble()) / reduction;
        inputs.alignerAppliedVoltage = alignerAppliedVoltage.getValueAsDouble();
        inputs.alignerSupplyCurrentAmps = alignerSupplyCurrent.getValueAsDouble();
        inputs.alignerTorqueCurrentAmps = alignerTorqueCurrent.getValueAsDouble();
        inputs.alignerTempCelsius = alignerTempCelsius.getValueAsDouble();

        inputs.beamBreakUnbroken = beamBreakSensor.get();
    }
    
    @Override 
    public void runVolts(double intakeVolts, double alignerVolts) {
        intakeTalon.setControl(voltageOut.withOutput(intakeVolts));
        alignerTalon.setControl(voltageOut.withOutput(alignerVolts));
    }
}
