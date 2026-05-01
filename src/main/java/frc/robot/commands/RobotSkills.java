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

/** Most of the robot is driven by InstantCommands in Robot Container */

public class RobotSkills
{
    private final RobotSkillsConstants constants = new RobotSkillsConstants();  

    /** Subsystems */
    private PoseEstimator s_PoseEstimator;
    private Swerve s_Swerve;
    //private Climber s_Climber;
    private FuelIndexer s_fuelIndexer;
    private FuelIntake s_fuelIntake;
    private FuelShooter s_fuelShooter;


    public RobotSkills(FuelShooter s_fuelShooter, FuelIndexer s_fuelIndexer, FuelIntake s_fuelIntake, Swerve s_Swerve)
    {
        this.s_fuelShooter = s_fuelShooter;
        this.s_fuelIndexer = s_fuelIndexer;
        this.s_fuelIntake = s_fuelIntake;
        this.s_Swerve = s_Swerve;
        //this.s_Climber = s_Climber;
    } 

    public Command shootShortFuel() 
    {
        return new ParallelCommandGroup(
        s_fuelShooter.slow(),
        new SequentialCommandGroup(
            new WaitCommand(1),
            s_fuelIndexer.slow()
        )
    );
    }

     public Command shootMidFuel() 
    {
        return new ParallelCommandGroup(
        s_fuelShooter.medium(),
        new SequentialCommandGroup(
            new WaitCommand(1),
            s_fuelIndexer.medium()
        )
    );
    }
    
    public Command shootLongFuel() 
    {
        return new ParallelCommandGroup(
        s_fuelShooter.fast(),
        new SequentialCommandGroup(
            new WaitCommand(1),
            s_fuelIndexer.fast()
        )
    );
    }

    public Command shootMidFuelAuto() 
    {
        return new ParallelCommandGroup(
        s_fuelShooter.medium(),
        new SequentialCommandGroup(
            new WaitCommand(1),
            s_fuelIndexer.medium()
        )
    ).withTimeout(5);
    }

    public Command shootShortFuelAuto() 
    {
        return new ParallelCommandGroup(
        s_fuelShooter.slow(),
        new SequentialCommandGroup(
            new WaitCommand(1),
            s_fuelIndexer.slow()
        )
    ).withTimeout(5);
    }

    public Command shootFast()
    {
        return new InstantCommand(() -> this.s_fuelShooter.fast());
    }

    public Command rollShort()
    {
        return new AutoDriveCommand
            (
                this.s_Swerve, "backward",
                constants.backwardsRollInches,
                constants.backwardsRollSeconds
            );
    }

    /*public Command ShootRollClimb()
    {
        return new SequentialCommandGroup
        (
            s_fuelShooter.fast(),
            new WaitCommand(3.0),
            s_fuelShooter.stopShooting(),
            Commands.runOnce(() ->  States.climberState = States.ClimberStates.ready),
            new AutoDriveCommand
            (
                this.s_Swerve, "backward",
                constants.backwardsRollInches,
                constants.backwardsRollSourceSeconds
            ),
            Commands.runOnce(() ->  States.climberState = States.ClimberStates.climb)
        );
    }*/

    public Command ShootRollClimb()
    {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                s_fuelShooter.fast(),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    s_fuelIndexer.fast()
                )
            ).withTimeout(3.0),

            Commands.runOnce(() -> States.climberState = States.ClimberStates.ready),
            new WaitCommand(2.0),

            new AutoDriveCommand(
                this.s_Swerve,
                "backward",
                constants.backwardsRollInches,
                constants.backwardsRollSourceSeconds
            ),

            Commands.runOnce(() -> States.climberState = States.ClimberStates.climb),
            new WaitCommand(3.0)
        );
    }
}
