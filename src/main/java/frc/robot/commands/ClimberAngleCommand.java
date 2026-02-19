package frc.robot.commands;
//import com.revrobotics.spark.ClosedLoopSlot;
//import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.States;
import frc.robot.subsystems.Climber;

public class ClimberAngleCommand extends Command
{
    private Climber climberHooks;
    private double currentClimberHooksAngleTarget;
    public ClimberAngleCommand(Climber climberHooks)
    {
        this.climberHooks = climberHooks;
        addRequirements(climberHooks);
    }

    @Override
    public void execute()
    {
        switch (States.climberState) 
        {
          case start:
            currentClimberHooksAngleTarget = ClimberConstants.Climber.CLIMBER_START_ANGLE;
            break;
          case ready:
            currentClimberHooksAngleTarget = ClimberConstants.Climber.CLIMBER_READY_ANGLE;            
            break;
          case climb:
            currentClimberHooksAngleTarget = ClimberConstants.Climber.CLIMBER_CLIMB_ANGLE;            
            break;
        } 
        climberHooks.setAngle(currentClimberHooksAngleTarget);
    }
}
