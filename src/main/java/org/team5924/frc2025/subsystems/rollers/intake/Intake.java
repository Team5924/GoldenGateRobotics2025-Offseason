package org.team5924.frc2025.subsystems.rollers.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum IntakeState {
        IN( // used for ground intake
            new LoggedTunableNumber("Intake/IntakeMotor/InVoltage", -6.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/InVoltage", -8.0)),
        SLOW_IN( // used for source intake (?) and holding inside
            new LoggedTunableNumber("Intake/IntakeMotor/SlowInVoltage", -2.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/SlowIn Voltage", -3.0)),
        TROUGH_OUT(
            new LoggedTunableNumber("Intake/IntakeMotor/TroughOutVoltage", 3.25), 
            new LoggedTunableNumber("Intake/AlignerMotor/TroughOutVoltage", 0.0)), 
        OUT(
            new LoggedTunableNumber("Intake/IntakeMotor/OutVoltage", 8.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/OutVoltage", 0.0)),
        OFF(
            new LoggedTunableNumber("Intake/IntakeMotor/OffVoltage", 0.0),
            new LoggedTunableNumber("Intake/AlignerMotor/OffVoltage", 0.0)),
        ALGAE_MODE_IDLE(
            new LoggedTunableNumber("Intake/IntakeMotor/AlgaeModeIdleVoltage", 0.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/AlgaeModeIdleVoltage", 0.0)),
        OPERATOR_CONTROL(
            new LoggedTunableNumber("Intake/IntakeMotor/OperatorControlVoltage", 0.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/OperatorControlVoltage", 0.0));

        public final DoubleSupplier intakeVoltage;
        public final DoubleSupplier alignerVoltage;

        IntakeState(LoggedTunableNumber intakeVoltage, LoggedTunableNumber alignerVoltage) {
            this.intakeVoltage = intakeVoltage;
            this.alignerVoltage = alignerVoltage;
        }
    }

    private final IntakeIO io;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.runVolts(RobotState.getInstance().getIntakeState().intakeVoltage.getAsDouble(), RobotState.getInstance().getIntakeState().alignerVoltage.getAsDouble());

        Logger.recordOutput("RobotState/IntakeState", RobotState.getInstance().getIntakeState());
    }

    public boolean isOperatorControlling() {
        IntakeState currentState = RobotState.getInstance().getIntakeState();
        return currentState.equals(IntakeState.OUT) || currentState.equals(IntakeState.TROUGH_OUT);
    }

    public void updateState() {
        if (isOperatorControlling() && !inputs.beamBreakUnbroken)
            RobotState.getInstance().setIntakeState(IntakeState.SLOW_IN);
    }

    public void setGoalState(IntakeState state) {
        RobotState.getInstance().setIntakeState(state);
    }
}
