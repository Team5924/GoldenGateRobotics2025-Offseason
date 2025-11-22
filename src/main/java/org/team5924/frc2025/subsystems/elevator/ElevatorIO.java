package org.team5924.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {

        /* Motor Data */
        public boolean motorIsConnected = true;
        public double motorPositionRads = 0.0;
        public double motorVelocityRadsPerSec = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorSupplyCurrentAmps = 0.0;
        public double motorTorqueCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;

        /* Real Elevator Data */
        public double realPos = 0.0;
        public double realVel = 0.0;
        public double realAccl = 0.0;

        /* Target Elevator Data */
        public double targetPos = 0.0;
        public double targetVel = 0.0;
        public double targetAccl = 0.0;

        public boolean minSoftStop = false;
        public boolean maxSoftStop = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    
    public default void periodicUpdates() {}

    public default void setHeight(double heightMeters) {}

    public default void setVoltage(double volts) {}

    public default void setSoftStopOn() {}

    public default void setSoftStopOff() {}
}
