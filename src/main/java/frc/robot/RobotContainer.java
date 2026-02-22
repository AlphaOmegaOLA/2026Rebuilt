package frc.robot;

/** Libraries for Autonomous mode */
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/** Controllers/Joystick libraries */
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControllerMap;

// Dashboard libraries
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Command-based libraries */
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.States;

// Subsysystem libraries
import frc.robot.subsystems.*;

// Camera Libraries
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    // Autonomous menu on dashboard
    private final SendableChooser<Command> autoChooser;

    // Controllers
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
	private final int rotationAxis = 4;

    // Outtake speed control
    private double outtakeSpeed = 1.0;


    // Microsoft Life cam on arm
    //private final UsbCamera usbcamera;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, ControllerMap.LOGO_RIGHT);
    private final JoystickButton dampen = new JoystickButton(driver, ControllerMap.RB);

    /* Operator Buttons */
    // X = Shoot Fuel
    private final JoystickButton b_shoot_Fuel = new JoystickButton(operator, XboxController.Button.kX.value);
    // Left Bumper = Climber Down
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
    private final FuelIntake s_FuelIntake = new FuelIntake();
    private final FuelIntakeArm s_FuelIntakeArm = new FuelIntakeArm();
    private final FuelIndexer s_FuelIndexer = new FuelIndexer();
    private final FuelShooter s_FuelShooter = new FuelShooter();

    /* Commands */
    // shootFuel spins up the indexer motor first and the shooter motor a half second later.
    private final Command c_shoot_Fuel = new RobotSkills(s_FuelShooter, s_FuelIndexer, s_FuelIntake, s_Swerve).shootFuel();

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


        // Start the fuel intake arm movement and set to the default start angle.
        s_FuelIntakeArm.setDefaultCommand(new FuelIntakeArmAngleCommand(s_FuelIntakeArm));

        // Start Camera */
        //usbcamera = CameraServer.startAutomaticCapture();
        //usbcamera.setResolution(640, 480);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. 
     */
    private void configureButtonBindings() 
    {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /* Operator Buttons */
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