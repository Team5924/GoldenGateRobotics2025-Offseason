package org.team5924.frc2025.subsystems.elevator;

import org.team5924.frc2025.util.LoggedTunableNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Elevator extends SubsystemBase{
    public enum ElevatorState{
        IDLE(new LoggedTunableNumber("ElevatorIdleHeight",0.0)),
        DOWN(new LoggedTunableNumber("ElevatorDownHeight",0.0)),
        HANDOFF(new LoggedTunableNumber("ElevatorCoralHandoffHeight",0.0)),
        LOLIPOP(new LoggedTunableNumber("ElevatorLolipopIntakeHeight",0.0)),
        L2(new LoggedTunableNumber("ElevatorL2Height",0.0)),
        L3(new LoggedTunableNumber("ElevatorL3Height",0.0)),
        L4(new LoggedTunableNumber("ElevatorL4Height",0.0)),
        SCORE_L2(new LoggedTunableNumber("ElevatorScoreL2Height",0.0)),    
        SCORE_L3(new LoggedTunableNumber("ElevatorScoreL3Height",0.0)),
        SCORE_L4(new LoggedTunableNumber("ElevatorScoreL4Height",0.0)),
        ALGAE_LOW(new LoggedTunableNumber("ElevatorAlgaeLowHeight",0.0)),
        ALGAE_HIGH(new LoggedTunableNumber("ElevatorAlgaeHighHeight",0.0)),
        ALGAE_GROUND(new LoggedTunableNumber("ElevatorAlgaeGroundIntakeHeight",0.0)),
        BARGE(new LoggedTunableNumber("ElevatorScoreBargeHeight",0.0)),
        PROCESSOR(new LoggedTunableNumber("ElevatorScoreProcessorHeight",0.0)),
        MANUAL(new LoggedTunableNumber("ElevatorManualVoltage",0.0));


        private final LoggedTunableNumber heightMeters;

        ElevatorState(LoggedTunableNumber heightMeters){
            this.heightMeters = heightMeters;
        }

    }

    @Getter private ElevatorState goalState;

    public void setGoalState(ElevatorState goalState){
        this.goalState = goalState;
        switch(goalState){
            case MANUAL:
                
        }
    }
}