package frc.robot.commands;
//import com.revrobotics.spark.ClosedLoopSlot;
//import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelIntakeArmConstants;
import frc.robot.States;
import frc.robot.subsystems.FuelIntakeArm;

public class FuelIntakeArmAngleCommand extends Command
{
    private FuelIntakeArm fuelIntakeArm;
    private double currentFuelIntakeArmAngleTarget;

    public FuelIntakeArmAngleCommand(FuelIntakeArm fuelIntakeArm)
    {
        this.fuelIntakeArm = fuelIntakeArm;
        addRequirements(fuelIntakeArm);
    }

    @Override
    public void execute()
    {
        switch (States.fuelIntakeArmAngleState) 
        {
          case start:
            currentFuelIntakeArmAngleTarget = FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_START_ANGLE;
            break;
          case intake:
            currentFuelIntakeArmAngleTarget = FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_GETFUEL_ANGLE;            
            break;
          default:
            currentFuelIntakeArmAngleTarget = FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_START_ANGLE;            
            break;
        } 
        fuelIntakeArm.setAngle(currentFuelIntakeArmAngleTarget);
    }
}
