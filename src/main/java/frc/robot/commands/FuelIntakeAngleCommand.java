package frc.robot.commands;
//import com.revrobotics.spark.ClosedLoopSlot;
//import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelIntakeAngleConstants;
import frc.robot.States;
import frc.robot.subsystems.FuelIntakeAngle;

public class FuelIntakeAngleCommand extends Command
{
    private FuelIntakeAngle fuelIntakeAngle;
    private double currentFuelIntakeAngleTarget;

    public FuelIntakeAngleCommand(FuelIntakeAngle fuelIntakeAngle)
    {
        this.fuelIntakeAngle = fuelIntakeAngle;
        addRequirements(fuelIntakeAngle);
    }

    @Override
    public void execute()
    {
        switch (States.fuelIntakeAngleState) 
        {
          case start:
            currentFuelIntakeAngleTarget = FuelIntakeAngleConstants.FuelIntakeAngle.FUEL_INTAKE_ANGLE_START_ANGLE;
            break;
          case intake:
            currentFuelIntakeAngleTarget = FuelIntakeAngleConstants.FuelIntakeAngle.FUEL_INTAKE_ANGLE;            
            break;
          default:
            currentFuelIntakeAngleTarget = FuelIntakeAngleConstants.FuelIntakeAngle.FUEL_INTAKE_ANGLE_START_ANGLE;            
            break;
        } 
        fuelIntakeAngle.setAngle(currentFuelIntakeAngleTarget);
    }
}
