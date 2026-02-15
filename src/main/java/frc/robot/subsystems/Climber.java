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
import com.revrobotics.RelativeEncoder;
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
  //private HardwareConfigs hardwareConfigs;
  public SparkClosedLoopController closedLoopController;
  public double currentCoralIntakeArmTarget = 0.0;

  public Climber() 
  {
    climberMotor = new SparkMax(ClimberConstants.Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    //hardwareConfigs = new HardwareConfigs();
    climberEncoder.setPosition(0);
    //fuelIntakeAngleMotor.configure(hardwareConfigs.coralAngleSparkConfig, SparkMax.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = climberMotor.getClosedLoopController();
  }
  
  public void setAngle(double angle)
  {
    closedLoopController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
    SmartDashboard.putNumber("FUEL INTAKE ANGLE", climberEncoder.getPosition());
  }
}
