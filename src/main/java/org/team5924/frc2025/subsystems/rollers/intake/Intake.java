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

public class Intake extends GenericRollerSystem<Intake.IntakeState>{

    @RequiredArgsConstructor
    @Getter
    public enum IntakeState implements VoltageState {
        IN(
            new LoggedTunableNumber("Intake/IntakeMotor/INVoltage", -6.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/INVoltage", -8.0)),
        SLOW_IN(
            new LoggedTunableNumber("Intake/IntakeMotor/SlowVoltage", -2.0), 
            new LoggedTunableNumber("Intake/AlignerMotor/SlowVoltage", -3.0)),
        TROUGH_OUT(
            new LoggedTunableNumber("Intake/IntakeMotor/Voltage", 3.25), 
            new LoggedTunableNumber("Intake/AlignerMotor/Voltage", 0.0)), 
        OUT(
            new LoggedTunableNumber("Intake/Motor/Voltage", 8.0), 
            new LoggedTunableNumber("Intake/Motor/Voltage", 0.0)),
        OFF(
            new LoggedTunableNumber("Intake/Motor/Voltage", 0.0),
            new LoggedTunableNumber("Intake/Motor/Voltage", 0.0)),
        ALGAE_MODE_IDLE(
            new LoggedTunableNumber("Intake/Motor/Voltage", 0.0), 
            new LoggedTunableNumber("Intake/Motor/Voltage", 0.0)),
        OPERATOR_CONTROL(
            new LoggedTunableNumber("Intake/Motor/Voltage", 0.0), 
            new LoggedTunableNumber("Intake/Motor/Voltage", 0.0));

        private final DoubleSupplier intakeVoltage;
        private final DoubleSupplier alignerVoltage;
        @Override
        public DoubleSupplier getVoltageSupplier() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getVoltageSupplier'");
        }
        
        
    }

    private IntakeState goalState;

    public Intake(IntakeIO io){
        super("Intake", io);
    }

      @Override
  public void periodic() {
    ((IntakeIO) io).runVolts(goalState.getIntakeVoltage().getAsDouble(),goalState.getAlignerVoltage().getAsDouble());
    super.periodic();

    // update CoralState periodically
    // updateCoralState();

    Logger.recordOutput("RobotState/Coral/InAndOutState", RobotState.getInstance().getIntakeState()); // TODO: Implement IntakeState
  }

    @Override
    public IntakeState getGoalState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getGoalState'");
    }
    
}
