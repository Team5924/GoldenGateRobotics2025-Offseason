package org.team5924.frc2025.subsystems.rollers.intake;

import org.team5924.frc2025.Constants;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOKrakenFOC;


public class IntakeIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements IntakeIO{

    private static final int intakeId = Constants.INTAKE_CAN_ID;
    private static final int alignerId = Constants.ALIGNER_CAN_ID;
    private static final String bus = Constants.INTAKE_OUT_BUS;
    private static final int currentLimitAmps = Constants.INTAKE_CURRENT_LIMIT;
    private static final boolean invert = Constants.INTAKE_INVERT;
    private static final boolean intakeBrake = Constants.INTAKE_BRAKE;
    private static final boolean alignerBrake = Constants.ALIGNER_BRAKE;
    private static final double reduction = Constants.INTAKE_REDUCTION;

    private class AlignerKrakenFOC extends GenericRollerSystemIOKrakenFOC{
        public AlignerKrakenFOC(int id, String bus, int currentLimitAmps, boolean invert, boolean brake, double reduction){
            super(id, bus, currentLimitAmps, invert, brake, reduction);
        }
    }

    private final AlignerKrakenFOC alignerSystem;


    public IntakeIOKrakenFOC(){
        super(intakeId, bus, currentLimitAmps, invert, intakeBrake, reduction);
        alignerSystem = new AlignerKrakenFOC(alignerId, bus, currentLimitAmps, invert, alignerBrake, reduction);
    }
    public void updateInputs(IntakeIOInputs inputs){
        //TODO: add beam breaker inputs
        super.updateInputs(inputs);
    }
    @Override 
    public void runVolts(double intakeVolts, double alignerVolts){
        super.runVolts(intakeVolts);
        alignerSystem.runVolts(alignerVolts);
    }
    
}
