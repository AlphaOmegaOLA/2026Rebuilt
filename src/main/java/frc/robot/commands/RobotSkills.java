package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotSkillsConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.States;
import frc.robot.Constants.*;

public class RobotSkills
{
    private final RobotSkillsConstants c_Constants = new RobotSkillsConstants();  

    /** Subsystems */
    private PoseEstimator s_PoseEstimator;
    private Swerve s_Swerve;
    private Climber s_Climber;
    private FuelFeeder s_fuelFeeder;
    private FuelIndexer s_fuelIndexer;
    private FuelIntake s_fuelIntake;
    private FuelShooter s_fuelShooter;


    public RobotSkills(FuelShooter s_fuelShooter, FuelIndexer s_fuelIndexer, FuelIntake s_fuelIntake, Swerve s_Swerve)
    {
        this.s_fuelShooter = s_fuelShooter;
        this.s_fuelIndexer = s_fuelIndexer;
        this.s_fuelIntake = s_fuelIntake;
        this.s_Swerve = s_Swerve;
    } 

    public Command shootFuel()
    {
        return new SequentialCommandGroup
        (
            s_fuelIndexer.fast(),
            new WaitCommand(0.5),
            s_fuelShooter.fast()
        );
    }

    /**
    public Command coralIntake()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.intake),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral0)
        );
    }

    public Command coralStart()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral0),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral0)
        );
    }

    public Command coral1()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral1),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral1)
        );
    }

    public Command coral2()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral2),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral2)
        );
    }

    public Command autoTroughCoral()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral2),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral2)
        );    
    }

    public Command shootFast()
    {
        return new InstantCommand(() -> this.coralShooter.fast());
    }

    public Command rollShortAndShoot()
    {
        return new SequentialCommandGroup
        (
            new AutoDriveCommand
            (
                this.swerve, "forward",
                constants.forwardRollInches,
                constants.backwardsRollSourceSeconds
            ),
            new ParallelCommandGroup(
                this.autoTroughCoral(),
                new SequentialCommandGroup(
                    new WaitCommand(2.0),
                    this.coralShooter.fast()
                )
            )
  
        );
    }
    */
}
