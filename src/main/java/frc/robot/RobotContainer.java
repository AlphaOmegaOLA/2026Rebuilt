package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControllerMap;
import frc.robot.States;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
//import frc.robot.subsystems.Elevator.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{

    /* Autonomous menu */
    private final SendableChooser<Command> autoChooser;

    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

   /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
	private final int rotationAxis = 4;

    private double outtakeSpeed = 0.35;


    // Microsoft Life cam on arm
    //private final UsbCamera usbcamera;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, ControllerMap.LOGO_RIGHT);
    //private final POVButton climber_up = new POVButton(driver, 0);
    //private final POVButton climber_down = new POVButton(driver, 180);
    private final JoystickButton dampen = new JoystickButton(driver, ControllerMap.RB);
    //private final JoystickButton alignButton = new JoystickButton(driver, ControllerMap.LB);

    /* Operator Buttons */
    // X = Shoot Fuel
    private final JoystickButton b_shoot_Fuel = new JoystickButton(operator, XboxController.Button.kX.value);
    // Y = Algae Intake
    //private final JoystickButton algae_intake = new JoystickButton(operator, XboxController.Button.kY.value);
    // Right Bumper = Algae Outtake
    //private final JoystickButton algae_outtake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // Left Bumper = Algae Spool In
    private final JoystickButton b_climber_Down = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // A = Move Fuel Intake Arm to the intake angle
    private final JoystickButton b_fuel_Arm_Intake_Angle = new JoystickButton(operator, XboxController.Button.kA.value);
    // B = Move Fuel Intake Arm to the starting angle. This is also the default when the robot initializes.
    private final JoystickButton b_fuel_Arm_Start_Angle = new JoystickButton(operator, XboxController.Button.kB.value);
    // Right Trigger = Climb Up
    private final Trigger b_climber_Up = new Trigger(() -> operator.getRightTriggerAxis() > 0.5);
    // Left Trigger = Coral2 Position and Angle
    private final Trigger b_climber_Ready = new Trigger(() -> operator.getLeftTriggerAxis() > 0.5);


    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    private final Climber s_Climber = new Climber();
    private final FuelFeeder s_FuelFeeder = new FuelFeeder();
    private final FuelIntake s_FuelIntake = new FuelIntake();
    private final FuelIntakeArm s_FuelIntakeArm = new FuelIntakeArm();
    private final FuelIndexer s_FuelIndexer = new FuelIndexer();
    private final FuelShooter s_FuelShooter = new FuelShooter();
    //private final RobotSkills coral = new RobotSkills(s_CoralIntakeArm, s_elevator, s_CoralIntakeShooter, s_Swerve);
    //private final RobotSkills autos = new RobotSkills(s_CoralIntakeArm, s_elevator, s_CoralIntakeShooter, s_Swerve);
    //private final Vision s_Vision = new Vision(s_PoseEstimator);
    //private final Command teleopAutoAlign = new LimelightAprilTagAlignCommand(s_Swerve, s_Vision);
    //private final Command autoAlign = new LimelightAutoAlignCommand(s_Swerve, s_Vision);
    

    /* Commands */
    //private final Command c_coralIntake = coral.coralIntake();
    //private final Command c_coral1 = coral.coral1();
    //private final Command c_coral2 = coral.coral2();
    private final Command c_shoot_Fuel = new RobotSkills(s_FuelShooter, s_FuelIndexer, s_FuelIntake, s_Swerve).shootFuel();

    /* AutoChooser */
    //private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {

        //NamedCommands.registerCommand("coral1", autos.coral1());
        //NamedCommands.registerCommand("coral2", autos.coral2());
        //NamedCommands.registerCommand("shootFast", autos.shootFast());

        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Mode", autoChooser);
        //autoChooser.setDefaultOption("1 Roll and Shoot", autos.rollShortAndShoot());
        //autoChooser.addOption("Limelight auto", autoAlign);
        //autoChooser.addOption("4 Note Long Auto", autos.fourNoteLongAuto());
        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> 0 // Dynamic heading placeholder
            )
        );

        /** Manual Joystick Intake/Outtake for Fuel */
        s_FuelIntake.setDefaultCommand(
           Commands.run(() -> s_FuelIntake.manual(operator.getLeftY() * outtakeSpeed), s_FuelIntake)
        );
        
        // The defaults elevator PID angle
        //s_elevator.setDefaultCommand(new PIDElevatorCommand(s_elevator));

        /** Start the fuel intake arm movement and set to the default start angle. */
        s_FuelIntakeArm.setDefaultCommand(new FuelIntakeArmAngleCommand(s_FuelIntakeArm));

        //s_elevator.setDefaultCommand(
          //  Commands.run(() -> s_elevator.manual(operator.getLeftY() * 0.2), s_elevator)
        //);

        // The default coral Arm PID angle
        //s_CoralIntakeArm.setDefaultCommand(new CoralIntakeArmCommand(s_CoralIntakeArm));

         /** Camera */
         //usbcamera = CameraServer.startAutomaticCapture();
        // usbcamera.setResolution(640, 480);

        // Configure the button bindings
        configureButtonBindings();
    }

        /*//Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }*/

    /**
     * Use this method to define your button->command mappings. 
     */
    private void configureButtonBindings() 
    {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //algae_off.whileTrue(new InstantCommand(() -> outtakeSpeed = 1.0));
        //algae_off.onFalse(new InstantCommand(() -> outtakeSpeed = 0.35));

        //climber_up.whileTrue(new InstantCommand(() -> s_Climber.manual(1.0)));
        //climber_up.onFalse(new InstantCommand (() -> s_Climber.manual(0.0)));
        //climber_down.whileTrue(new InstantCommand(() -> s_Climber.manual(-1.0)));
        //climber_down.onFalse(new InstantCommand (() -> s_Climber.manual(0.0)));

        /* Operator Buttons */
        //algaeSpool_out.whileTrue(new InstantCommand (() -> s_AlgaeSpool.intakeDown(-1.0)));
        //algaeSpool_out.onFalse(new InstantCommand(() -> s_AlgaeSpool.intakeStop()));
        //algae_intake.whileTrue(new InstantCommand(() -> s_AlgaeIntakeShooter.intake(1.0)));
        //algae_intake.onFalse(new InstantCommand(() -> s_AlgaeIntakeShooter.manual(0.0)));
        //algae_outtake.whileTrue(new InstantCommand(() -> s_AlgaeIntakeShooter.outtake(-1.0)));
        //algae_outtake.onFalse(new InstantCommand(() -> s_AlgaeIntakeShooter.manual(0.0)));
        //algaeSpool_in.whileTrue(new InstantCommand(() -> s_AlgaeSpool.intakeUp(1.0)));
        //algaeSpool_in.onFalse(new InstantCommand(() -> s_AlgaeSpool.intakeStop()));
        //coral_Intake.onTrue(c_coralIntake);
        //coral_Intake.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL INTAKE BUTTON")));

        //coral_Start.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL START BUTTON")));
        //coral_coral1.onTrue(c_coral1);
        //coral_coral1.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL CORAL1 BUTTON")));
        //coral_coral2.onTrue(c_coral2);
        //coral_coral2.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL C0RAL2 BUTTON")));
        //alignButton.whileTrue(teleopAutoAlign);
        // Climber Ready Position
        b_climber_Ready.onTrue(new InstantCommand(() -> States.climberState = States.ClimberStates.ready));
        // Climb!
        b_climber_Up.onTrue(new InstantCommand(() -> States.climberState = States.ClimberStates.climb));
        // Climber Down
        b_climber_Down.onTrue(new InstantCommand(() -> States.climberState = States.ClimberStates.start));
        // Set intake arm to the intake angle
        b_fuel_Arm_Intake_Angle.onTrue(new InstantCommand(() -> States.fuelIntakeArmAngleState = States.FuelIntakeArmStates.intake));
        // Set intake arm to the starting angle. This is also the default when the robot intitializes. 
        b_fuel_Arm_Start_Angle.onTrue(new InstantCommand(() -> States.fuelIntakeArmAngleState = States.FuelIntakeArmStates.start));
        // Shoot fuel
        b_shoot_Fuel.whileTrue(c_shoot_Fuel);
        
        /** Log button presses to dashboard for Dashboard for debugging */
        b_climber_Ready.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CLIMBER READY BUTTON")));
        b_climber_Up.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CLIMBER UP BUTTON")));
        b_climber_Down.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CLIMBER DOWN BUTTON")));
        b_fuel_Arm_Intake_Angle.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "INTAKE ARM INTAKE BUTTON")));
        b_fuel_Arm_Start_Angle.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "INTAKE ARM START BUTTON")));
        b_shoot_Fuel.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "SHOOT FUEL BUTTON")));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
        //return autoChooser.getSelected();
    }
}