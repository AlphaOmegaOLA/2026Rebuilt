package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.spark.config.SparkMaxConfig;

/** I think these hardware configs are deprecated, but we may need to set these up for each
 *  subsystem if I'm wrong. We can use our 2025 code as a model if things aren't working right
 *  and we need to do that: 
 *  https://raw.githubusercontent.com/AlphaOmegaOLA/2025Reefscape/refs/heads/main/src/main/java/frc/robot/HardwareConfigs.java
 */

public final class HardwareConfigs 
{
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public SparkMaxConfig swerveAngleSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig swerveDriveSparkConfig =  new SparkMaxConfig();

    public HardwareConfigs()
    {
        /** Swerve CANCoder Configuration */
       swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

       //Swerve angle motor config
       //Motor inverts and nuetral modes
       swerveAngleSparkConfig.inverted(Constants.Swerve.angleMotorInvert);
       swerveAngleSparkConfig.idleMode(Constants.Swerve.angleNeutralMode);

       //Gear ratio and wrapping config
       swerveAngleSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
       swerveAngleSparkConfig.encoder.velocityConversionFactor((360.0 / Constants.Swerve.angleGearRatio) / 60.0);
       swerveAngleSparkConfig.closedLoop.positionWrappingEnabled(true);
       //swerveAngleSparkConfig.closedLoop.positionWrappingInputRange(0.0, 360.0);

       //current limiting
       swerveAngleSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveAngleSparkConfig.closedLoop.p(Constants.Swerve.angleKP);
       swerveAngleSparkConfig.closedLoop.i(Constants.Swerve.angleKI);
       swerveAngleSparkConfig.closedLoop.d(Constants.Swerve.angleKD);

       //Swerve drive motor config
       //Motor inverts and nuetral modes
       swerveDriveSparkConfig.inverted(Constants.Swerve.driveMotorInvert);
       swerveDriveSparkConfig.idleMode(Constants.Swerve.driveNeutralMode);

       //Gear ratio and wrapping config
       swerveDriveSparkConfig.encoder.positionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio);
       swerveDriveSparkConfig.encoder.velocityConversionFactor((Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio) / 60.0);
       swerveDriveSparkConfig.closedLoop.positionWrappingEnabled(true);

       //current limiting
       swerveDriveSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
       swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
       swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

       swerveAngleSparkConfig.openLoopRampRate(Constants.Swerve.openLoopRamp);
       swerveAngleSparkConfig.closedLoopRampRate(Constants.Swerve.closedLoopRamp);
    }

}