package org.team5924.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {

    /* Motor Data */
    public boolean motorConnected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    /* Real Elevator Data */
    public double realPos = 0.0;
    public double realVel = 0.0;
    public double realAccl = 0.0;

    /* Motion Target Elevator Data */
    public double targetPos = 0.0;
    public double targetVel = 0.0;

    public double setpointMeters = 0.0;

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
