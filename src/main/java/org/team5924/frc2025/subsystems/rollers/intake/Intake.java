package org.team5924.frc2025.subsystems.rollers.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem.VoltageState;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOutIO;
import org.team5924.frc2025.util.LoggedTunableNumber;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Intake extends GenericRollerSystem<Intake.IntakeState> {
    
    @RequiredArgsConstructor
    @Getter
    public enum IntakeState implements VoltageState {
        IN(
            new LoggedTunableNumber("Intake/IntakeMotor/InVoltage", -6.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/InVoltage", -8.0)),
        SLOW_IN(
            new LoggedTunableNumber("Intake/IntakeMotor/SlowInVoltage", -2.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/SlowInVoltage", -3.0)),
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

        private final DoubleSupplier intakeVoltage;
        private final DoubleSupplier alignerVoltage;
    }

    private IntakeState goalState;

    public Intake(IntakeIO io) {
        super("Intake", io);
    }

    @Override
    public void periodic() {
        ((IntakeIO) io).runVolts(goalState.getIntakeVoltage().getAsDouble(), goalState.getAlignerVoltage().getAsDouble());
        super.periodic();

        Logger.recordOutput("RobotState/Intake", RobotState.getInstance().getIntakeState());
    }
}
