// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
//import frc.robot.HardwareConfigs;
//import frc.robot.Robot;

public class Climber extends SubsystemBase 
{
  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private SparkMaxConfig climberMotorConfig;
  public SparkClosedLoopController closedLoopController;
  public double currentClimberTarget = 0.0;

  public Climber() 
  {
    double degreesPerMotorRotation = 360.0 / ClimberConstants.Climber.CLIMBER_MOTOR_GEAR_RATIO;

    climberMotor = new SparkMax(ClimberConstants.Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig.encoder.positionConversionFactor(degreesPerMotorRotation);
    climberMotorConfig.encoder.velocityConversionFactor(degreesPerMotorRotation / 60);
    climberMotorConfig.closedLoop.p(ClimberConstants.Climber.CLIMBER_P);
    climberMotorConfig.closedLoop.i(ClimberConstants.Climber.CLIMBER_I);
    climberMotorConfig.closedLoop.d(ClimberConstants.Climber.CLIMBER_D);
    climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0);
    closedLoopController = climberMotor.getClosedLoopController();
  }
  
  public void setAngle(double angle)
  {
    currentClimberTarget = angle;
    closedLoopController.setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getAngle() 
  {
    return climberEncoder.getPosition();
  }
 
 //Check if this is the correct method to get the angle
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CLIMBER ANGLE", climberEncoder.getPosition());
    SmartDashboard.putNumber("CLIMBER TARGET", currentClimberTarget);
    SmartDashboard.putNumber("CLIMBER APPLIED OUTPUT", climberMotor.getAppliedOutput());
  }
}
