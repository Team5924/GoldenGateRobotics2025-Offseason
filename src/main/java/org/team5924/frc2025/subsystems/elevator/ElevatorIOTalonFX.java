package org.team5924.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANdi;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO {
  /* Motor Hardware */
  private final TalonFX talon;

  /* Sensor Hardware */
  private final CoreCANdi elevatorCANdi;

  /* Configurators */
  private TalonFXConfigurator leaderTalonConfig;

  /* Configs */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpoint;
  private final CANcoderConfiguration canCoderConfig;

  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.00);
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.13);
  LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", .44);
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 7);
  LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.07);
  LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.34);

  LoggedTunableNumber motionAcceleration = new LoggedTunableNumber("Elevator/MotionAcceleration", 400);
  LoggedTunableNumber motionCruiseVelocity = new LoggedTunableNumber("Elevator/MotionCruiseVelocity", 400);
  LoggedTunableNumber motionJerk = new LoggedTunableNumber("Elevator/MotionJerk", 1000);

  /* Status Signals */
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private StatusSignal<Double> closedLoopReferenceSlope;
  double prevClosedLoopReferenceSlope = 0.0;
  double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageControl;
  private final MotionMagicVoltage magicMotionVoltage;

  /* Alerts */
  private final Alert updateMotorConfigAlert =
      new Alert("Update elevator motor config error!", Alert.AlertType.kWarning);

  private final Alert initalMotorConfigAlert =
      new Alert(
          "Initial elevator motor config error! Restart robot code to clear.",
          Alert.AlertType.kError);

  private final Alert candiPin1FloatAlert =
      new Alert("Elevator CANdiPin1 is floating. Check connection.", Alert.AlertType.kWarning);

  private final Alert candiPin2FloatAlert =
      new Alert("Elevator CANdiPin2 is floating. Check connection.", Alert.AlertType.kWarning);


  public ElevatorIOTalonFX() {
    TalonFXConfigurator talonConfig = new TalonFXConfigurator(null);
    talon = new TalonFX(Constants.ELEVATOR_CAN_ID);
    elevatorCANdi = new CANdi(Constants.ELEVATOR_CANDI_ID);

    leaderTalonConfig = talon.getConfigurator();

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kG = kG.get();

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.MOTOR_TO_ELEVATOR_REDUCTION;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // feedbackConfigs.FeedbackRemoteSensorID = Constants.ELEVATOR_CANCODER_ID;
    // feedbackConfigs.SensorToMechanismRatio = Constants.CANCODER_TO_ELEVATOR_REDUCTION;
    feedbackConfigs.RotorToSensorRatio = Constants.MOTOR_TO_ELEVATOR_REDUCTION;

    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = Constants.ELEVATOR_CANCODER_OFFSET;

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[14];

    statusArray[0] = talonConfig.apply(Constants.ELEVATOR_CONFIG);
    statusArray[1] = talonConfig.apply(slot0Configs);
    statusArray[2] = talonConfig.apply(motionMagicConfigs);
    statusArray[3] = talonConfig.apply(openLoopRampsConfigs);
    statusArray[4] = talonConfig.apply(closedLoopRampsConfigs);
    statusArray[5] = talonConfig.apply(feedbackConfigs);
    statusArray[6] = elevatorCANdi.getConfigurator().apply(Constants.ELEVATOR_CANDI_CONFIGS);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;
    initalMotorConfigAlert.set(isErrorPresent);
    Logger.recordOutput("Elevator/InitConfReport", statusArray);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    closedLoopReferenceSlope = talon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        closedLoopReferenceSlope);


    voltageControl =
        new VoltageOut(0)
            .withUpdateFreqHz(0.0)
            .withEnableFOC(true)
            .withLimitForwardMotion(elevatorCANdi.getS2Closed().getValue());
    magicMotionVoltage =
        new MotionMagicVoltage(0)
            .withEnableFOC(true)
            .withLimitForwardMotion(elevatorCANdi.getS2Closed().getValue());

    talon.setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVolts,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.positionRads = position.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelsius = tempCelsius.getValue().in(Celsius);

    inputs.realPos = getHeight();
    inputs.realVel = getVelocity();

    inputs.targetVel =
        rotationsToMeters(talon.getClosedLoopReferenceSlope().getValue());     
    inputs.targetPos =
        rotationsToMeters(talon.getClosedLoopReference().getValue());

    inputs.setpointMeters = setpoint;

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.realAccl =
          (inputs.targetVel - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.targetVel;   //Hi h
    prevReferenceSlopeTimestamp = currentTime;

    inputs.minSoftStop = elevatorCANdi.getS1Closed().getValue();
    inputs.maxSoftStop = elevatorCANdi.getS2Closed().getValue();
  }

  @Override
  public void periodicUpdates() {
    updateTunableNumbers();
    isAtZero();

    candiPin1FloatAlert.set(elevatorCANdi.getS1State().getValue() == S1StateValue.Floating);
    candiPin2FloatAlert.set(elevatorCANdi.getS2State().getValue() == S2StateValue.Floating);
  }

  public void updateTunableNumbers() {
    if (kA.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || motionAcceleration.hasChanged(hashCode())
        || motionCruiseVelocity.hasChanged(hashCode())) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();
      slot0Configs.kG = kG.get();

      motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      StatusCode[] statusArray = new StatusCode[4];

      statusArray[0] = leaderTalonConfig.apply(slot0Configs);
      statusArray[1] = leaderTalonConfig.apply(motionMagicConfigs);;

      boolean isErrorPresent = false;
      for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;
      updateMotorConfigAlert.set(isErrorPresent);
      Logger.recordOutput("Elevator/Leader/UpdateConfReport", statusArray);
    }
  }

  @Override
  public void setHeight(double heightMeters) {
    if (!DriverStation.isEnabled()) {
      talon.setControl(new VoltageOut(0.0));
      return;
    }

    setpoint = heightMeters;
    talon.setControl(magicMotionVoltage.withPosition(metersToRotations(heightMeters)));
    Logger.recordOutput("Elevator/GoalHeight", heightMeters);
  }

  @Override
  public void setVoltage(double volts) {
    Logger.recordOutput("Elevator/RequestedVoltsManual", volts);
    talon.setControl(voltageControl.withOutput(volts));
  }

  public boolean isAtZero() {
    if (elevatorCANdi.getS1Closed().getValue()) {
      talon.setPosition(0.0);
      return true;
    }

    return false;
  }

  public double rotationsToMeters(double rotations) {
    return (rotations
            * 2
            * Math.PI
            * Constants.ELEVATOR_SPROCKET_RADIUS.in(Meters)
            / Constants.MOTOR_TO_ELEVATOR_REDUCTION)
        * 2; // Account for cascade rigging
  }

  public static double metersToRotations(double height) {
    return height
        * Constants.MOTOR_TO_ELEVATOR_REDUCTION
        / (2 * Math.PI * Constants.ELEVATOR_SPROCKET_RADIUS.in(Meters))
        / 2; // Account for cascade rigging
  }

  private double getHeight() {
    return rotationsToMeters(talon.getPosition().getValueAsDouble());
  }

  private double getVelocity() {
    return rotationsToMeters(talon.getVelocity().getValueAsDouble());
  }
}
