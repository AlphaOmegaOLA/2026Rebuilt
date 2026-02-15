package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelShooter;

public class FuelShooterCommand extends Command
{
    private FuelShooter fuelShooter;

    public FuelShooterCommand(FuelShooter fuelShooter)
    {
        this.fuelShooter = fuelShooter;
        addRequirements(fuelShooter);
    }

    public Command shootSlow()
    {
       return this.fuelShooter.slow();
    }

    public Command shootFast()
    {
        return this.fuelShooter.fast();
    }
}
