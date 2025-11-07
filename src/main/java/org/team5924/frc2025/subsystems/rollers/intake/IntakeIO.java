package org.team5924.frc2025.subsystems.rollers.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    abstract class IntakeIOInputs {
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

        public boolean beamBreakConnected = true;
        public boolean beamBreakUnbroken = false;
    }

    default void runVolts(double intakeVolts, double alignerVolts){}
}
