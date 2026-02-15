package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.SwerveModuleConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants.driveGearRatios;

public final class Constants 
{
    public static final double stickDeadband = 0.1;

    public static final class Swerve 
    {
        public static final int pigeonID = 18;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSNeoSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSNeoSwerveConstants.SDSMK4i(driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        // Did we update these measurements?
        public static final double trackWidth = Units.inchesToMeters(24); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final double driveRevToMeters =  wheelCircumference / (chosenModule.driveGearRatio);
        public static final double driveRpmToMetersPerSecond = driveRevToMeters/60 ;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

         /* Motor Inverts */
         public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
         public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive Motor to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.2;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        // Did we update this yet?
        public static final double driveKP = 0.012; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

         /* Heading PID Values */
        public static final double HeadingKP = 0.5;
        public static final double HeadingKI = 0.0;
        public static final double HeadingKD = 0;
        public static final double HeadingTolerence = 0;

        //Motor power gain
        public static final double drivePower = 1;
        public static final double anglePower = .9;


        /* Drive Motor Characterization Values from SysID */
        // Did we update this yet?
        public static final double driveKS = (0.32); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51);
        public static final double driveKA = (0.27);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final  IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final  IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.450439);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 15;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.103516);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.149170);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.117920);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public final class FuelIntakeConstants
    {
        public static final double FULL = 1.0;
        public static final double HALF = 0.5;
        public static final double QUARTER = 0.25;
        
        public static final class FuelIntake
        {
            public static final int FUEL_INTAKE_MOTOR_LEFT_ID = 11;
            public static final int FUEL_INTAKE_MOTOR_RIGHT_ID = 90;
            public static final boolean FUEL_INTAKE_MOTOR_LEFT_INVERTED = false;
            public static final boolean FUEL_INTAKE_MOTOR_RIGHT_INVERTED = false;
            public static final  IdleMode FUEL_INTAKE_NEUTRAL_MODE = IdleMode.kBrake;
            public static final double FUEL_INTAKE_GEAR_RATIO = 1.0;
            public static final double FULL_SPEED = FULL;
            public static final double HALF_SPEED = HALF;
            public static final double QUARTER_SPEED = QUARTER;
        }
    }

    public final class FuelFeederConstants
    {
        public static final double FULL = 1.0;
        public static final double HALF = 0.5;
        public static final double QUARTER = 0.25;
        
        public static final class fuelFeeder
        {
            public static final int FUEL_FEEDER_MOTOR_ID = 11;
            public static final boolean FUEL_FEEDER_MOTOR_INVERTED = false;
            public static final  IdleMode FUEL_FEEDER_NEUTRAL_MODE = IdleMode.kBrake;
            public static final double FUEL_INTAKEFUEL_FEEDER_GEAR_RATIO = 1.0;
            public static final double FULL_SPEED = FULL;
            public static final double HALF_SPEED = HALF;
            public static final double QUARTER_SPEED = QUARTER;
        }
    }

    public final class FuelShooterConstants
    {
        public static final double FULL = 1.0;
        public static final double HALF = 0.5;
        public static final double QUARTER = 0.25;
        
        public static final class FuelShooter
        {
            public static final int FUEL_SHOOTER_MOTOR_LEFT_ID = 11;
            public static final int FUEL_SHOOTER_MOTOR_RIGHT_ID = 90;
            public static final boolean FUEL_SHOOTER_MOTOR_LEFT_INVERTED = false;
            public static final boolean FUEL_SHOOTER_MOTOR_RIGHT_INVERTED = false;
            public static final  IdleMode FUEL_SHOOTER_NEUTRAL_MODE = IdleMode.kBrake;
            public static final double FUEL_SHOOTER_GEAR_RATIO = 1.0;
            public static final double FULL_SPEED = FULL;
            public static final double HALF_SPEED = HALF;
            public static final double QUARTER_SPEED = QUARTER;
        }
    }

    public final class ClimberConstants
    {
        public static final double FULL = 1.0;
        public static final double HALF = 0.5;
        public static final double QUARTER = 0.25;
        
        public static final class Climber
        {
             // ARM MOTOR IDS AND SPEEDS
             public static final int CLIMBER_MOTOR_ID = 5;
             public static final boolean CLIMBER_MOTOR_INVERTED = false;
             public static final  IdleMode CLIMBER_NEUTRAL_MODE = IdleMode.kBrake;
             public static final double FULL_SPEED = FULL;
             public static final double HALF_SPEED = HALF;
             public static final double QUARTER_SPEED = QUARTER;
             // Uses the through bore encoder
             //public static final int CORAL_ANGLE_ENCODER_ID = 0;
         
             // CORAL ANGLES
             // Need to determine the starting offset angle of the
             // Rev Through Bore Encoder and set it here:
             public static final Rotation2d CLIMBER_ENCODER_OFFSET = Rotation2d.fromDegrees(350+97);
             public static final double CLIMBER_MOTOR_GEAR_RATIO = 20.0;
             public static final double CLIMBER_START_ANGLE = -30.0;
             public static final double CLIMBER_READY_ANGLE = -30.0;
             public static final double CLIMBER_CLIMB_ANGLE = -180.0;
     
             // ELEVATOR PID - we will tune these values!!
             public static final int CLIMBER_ANGLE_CURRENT_LIMIT = 50;
             public static final double CLIMBER_UPDATE_OUTPUT = .02;
             public static final double CLIMBER_MAX_VELOCITY = 1.75;
             public static final double CLIMBER_MAX_ACCELERATION = .75;
             public static final double CLIMBER_P = 0.005;
             public static final double CLIMBER_I = 0.0;
             public static final double CLIMBER_D = 0.005;
             public static final double CLIMBER_S = 1.1;
             public static final double CLIMBERE_G = 1.2;
             public static final double CLIMBER_V = 1.3;
             public static final double ERROR_TOLERANCE = 0.1;
        }
    }

    public final class FuelIntakeAngleConstants
    {
        public static final double FULL = 1.0;
        public static final double HALF = 0.5;
        public static final double QUARTER = 0.25;
        
        public static final class FuelIntakeAngle
        {
             // ARM MOTOR IDS AND SPEEDS
             public static final int FUEL_INTAKE_ANGLE_MOTOR_ID = 5;
             public static final boolean FUEL_INTAKE_ANGLE_MOTOR_INVERTED = false;
             public static final  IdleMode FUEL_INTAKE_ANGLE_NEUTRAL_MODE = IdleMode.kBrake;
             public static final double FULL_SPEED = FULL;
             public static final double HALF_SPEED = HALF;
             public static final double QUARTER_SPEED = QUARTER;
             // Uses the through bore encoder
             //public static final int CORAL_ANGLE_ENCODER_ID = 0;
         
             // CORAL ANGLES
             // Need to determine the starting offset angle of the
             // Rev Through Bore Encoder and set it here:
             public static final Rotation2d FUEL_INTAKE_ANGLE_ENCODER_OFFSET = Rotation2d.fromDegrees(350+97);
             public static final double FUEL_INTAKE_ANGLE_MOTOR_GEAR_RATIO = 20.0;
             public static final double FUEL_INTAKE_ANGLE_START_ANGLE = -30.0; // We hope this hits the tray
             public static final double FUEL_INTAKE_ANGLE = -180.0;
     
             // ELEVATOR PID - we will tune these values!!
             public static final int FUEL_INTAKE_ANGLE_CURRENT_LIMIT = 50;
             public static final double FUEL_INTAKE_ANGLE_UPDATE_OUTPUT = .02;
             public static final double FUEL_INTAKE_ANGLE_MAX_VELOCITY = 1.75;
             public static final double FUEL_INTAKE_ANGLE_MAX_ACCELERATION = .75;
             public static final double FUEL_INTAKE_ANGLE_P = 0.005;
             public static final double FUEL_INTAKE_ANGLE_I = 0.0;
             public static final double FUEL_INTAKE_ANGLE_D = 0.005;
             public static final double FUEL_INTAKE_ANGLE_S = 1.1;
             public static final double FUEL_INTAKE_ANGLE_G = 1.2;
             public static final double FUEL_INTAKE_ANGLE_V = 1.3;
             public static final double ERROR_TOLERANCE = 0.1;
        }
    }

    public static final class AutoConstants 
    { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        //TODO: Must be tuned to specific robot
        public static final PIDConstants translationPID = new PIDConstants(0, 0, 0);
        public static final PIDConstants rotationPID = new PIDConstants(0, 0, 0);

        //TODO: Must be tuned to specific robot
        public static final double ROBOT_MASS_KG = 74.088;
        public static final double ROBOT_MOI = 6.883;
        public static final double WHEEL_COF = 1.2;

        public static final ModuleConfig moduleConfig = new ModuleConfig(
                (Constants.Swerve.chosenModule.wheelDiameter / 2),
                (Constants.Swerve.maxSpeed),
                Constants.AutoConstants.WHEEL_COF,
                DCMotor.getNEO(1).withReduction(Constants.Swerve.chosenModule.driveGearRatio),
                Constants.Swerve.driveCurrentThreshold,
              1);
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class PoseEstimator {
        public static final Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3,N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

}