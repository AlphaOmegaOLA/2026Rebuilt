package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelIndexer;

public class FuelIndexerCommand extends Command
{
    private FuelIndexer fuelIndexer;
    
    public FuelIndexerCommand(FuelIndexer fuelIndexer)
    {
        this.fuelIndexer = fuelIndexer;
        addRequirements(fuelIndexer);
    }

    public Command indexSlow()
    {
       return this.fuelIndexer.slow();
    }

    public Command indexFast()
    {
        return this.fuelIndexer.fast();
    }
}
