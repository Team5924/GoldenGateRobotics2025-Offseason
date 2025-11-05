package org.team5924.frc2025.subsystems.pivot;

import lombok.Getter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.Elastic.Notification;
import org.team5924.frc2025.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2025.util.LoggedTunableNumber;



public class IntakePivot extends SubsystemBase{

    private final IntakePivotIO io;

    public static final LoggedTunableNumber PIV_POS_TOLERANCE = 
        new LoggedTunableNumber("IntakePivot/PosTolerance",0.02);
    private final IntakePivotIOInputsAutoLogged pivotInput = new IntakePivotIOInputsAutoLogged();


    // Intake Preset Positions
    public enum IntakePivotState {
        INTAKE_FLOOR(new LoggedTunableNumber("IntakePivotFloorRads",0)),
        INTAKE_STATION(new LoggedTunableNumber("IntakePivotStationRads",0)),
        INTAKE_FEED(new LoggedTunableNumber("IntakePivotFeedRads",0)),
        INTAKE_LOW_ARM(new LoggedTunableNumber("IntakePivotLoweredArmRads",0)),
        SCORE_TROUGH(new LoggedTunableNumber("IntakePivotTroughScoreRads",0)),
        MOVING(new LoggedTunableNumber("IntakePivotMoving",0)),
        OPERATOR_CONTROL(new LoggedTunableNumber("IntakePivotOperatorRads",0));
        
        private final LoggedTunableNumber rads;

        IntakePivotState(LoggedTunableNumber rads) {
            this.rads = rads;
        }

    }

    @Getter private IntakePivotState goalState;

    private final Alert intakePivotMotorDisconnected;

    private final Notification intakePivotMotorDisconnectedNotification;

    public IntakePivot(IntakePivotIO io){
        this.io = io;
        this.goalState = IntakePivotState.INTAKE_FEED;
        this.intakePivotMotorDisconnected =
            new Alert("Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);
        this.intakePivotMotorDisconnectedNotification = 
            new Notification(NotificationLevel.WARNING, "Intake Pivot Warning", "Intake Pivot Motor Disconnected");
    }

    @Override
    public void periodic(){
        io.updateInputs(pivotInput);
        Logger.processInputs("IntakePivot", pivotInput);

        Logger.recordOutput("IntakePivot/GoalState", goalState.toString());
        Logger.recordOutput("IntakePivot/TargetRads", goalState.rads);

        intakePivotMotorDisconnected.set(!pivotInput.intakePivotMotorConnected);

        /* if (!pivotInput.intakePivotMotorConnected){
            Elastic.sendNotification(intakePivotMotorDisconnectedNotification);
        } */

    }


    public double getIntakePivotPosRads(){
        return pivotInput.intakePivotPositionRads / Constants.MOTOR_TO_INTAKE_PIVOT_REDUCTION;
    }

    public boolean isAtSetpoint(){
        return Math.abs(getIntakePivotPosRads() - this.goalState.rads.getAsDouble()) < PIV_POS_TOLERANCE.getAsDouble();
    }
    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }
    public void setGoalState(IntakePivotState goalState){
        this.goalState = goalState;
        switch(goalState){
            case OPERATOR_CONTROL:
                RobotState.getInstance().setIntakePivotState(IntakePivotState.OPERATOR_CONTROL);
                break;
            case MOVING:
                DriverStation.reportError("Invalid goal IntakePivotState!", null);
                break;
            default:
                RobotState.getInstance().setIntakePivotState(IntakePivotState.MOVING);
                io.setPosition(goalState.rads.getAsDouble());
                break;
        }
    }

}
