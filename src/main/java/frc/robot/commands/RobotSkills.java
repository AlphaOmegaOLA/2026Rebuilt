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
    private Climber s_Climber;
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

    public Command ShootRollClimb()
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
    }
}
