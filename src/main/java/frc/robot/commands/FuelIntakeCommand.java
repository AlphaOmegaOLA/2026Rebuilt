package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelIntake;

public class FuelIntakeCommand extends Command
{
    private FuelIntake fuelIntake;
    
    public FuelIntakeCommand(FuelIntake fuelIntake)
    {
        this.fuelIntake = fuelIntake;
        addRequirements(fuelIntake);
    }

    public Command intakeSlow()
    {
       return this.fuelIntake.slow();
    }

    public Command intakeFast()
    {
        return this.fuelIntake.fast();
    }
}
