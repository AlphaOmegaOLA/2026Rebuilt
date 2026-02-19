package frc.robot;

public class States 
{

    public static enum DriveStates 
    {
        standard, leftHold, rightHold, forwardHold, backwardHold, DynamicLock
    }

    public static enum AlignedStates 
    {
        aligned, unAligned, normal
    }

    public static enum ClimberStates 
    {
       start, ready, climb
    }

    public static enum FuelIntakeArmStates 
    {
       start, intake
    }

    public static enum CoralIntakeArmStates 
    {
        coral0, coral1, coral2, intake
    }

    public static DriveStates driveState = DriveStates.standard;
    public static AlignedStates alignedState = AlignedStates.normal;
    public static ClimberStates climberState = ClimberStates.start;
    public static FuelIntakeArmStates fuelIntakeArmAngleState = FuelIntakeArmStates.start; 
    public static CoralIntakeArmStates coralIntakeArmState = CoralIntakeArmStates.coral0;
}
