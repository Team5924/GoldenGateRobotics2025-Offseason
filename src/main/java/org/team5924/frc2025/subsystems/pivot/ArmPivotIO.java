package org.team5924.frc2025.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface ArmPivotIO {

    @AutoLog
    public class ArmPivotIOInputs{
        public boolean armPivotMotorConnected = true;

        public double armPivotPositionRads = 0;
        public double armPivotVelocityRadsPerSec = 0;
        public double armPivotAppliedVolts = 0;
        public double armPivotSupplyCurrentAmps = 0;
        public double armPivotTorqueCurrentAmps = 0;
        public double armPivotTempCelsius = 0;

        public boolean armPivotCANcoderConnected = true;
        public double armPivotCANcoderRelativePositionRads = 0;
        public double armPivotCANcoderAbsolutePositionRads = 0;
    }
    
    public default void updateInputs(ArmPivotIOInputs input){}

    public default void setVoltage(double volts){}

    public default void setPosition(double rads){}

    public default void stop(){}
}
