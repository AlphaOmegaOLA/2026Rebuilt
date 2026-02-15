package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelFeeder;

public class FuelFeederCommand extends Command
{
    private FuelFeeder fuelFeeder;
    
    public FuelFeederCommand(FuelFeeder fuelFeeder)
    {
        this.fuelFeeder = fuelFeeder;
        addRequirements(fuelFeeder);
    }

    public Command feedSlow()
    {
       return this.fuelFeeder.slow();
    }

    public Command feedFast()
    {
        return this.fuelFeeder.fast();
    }
}
