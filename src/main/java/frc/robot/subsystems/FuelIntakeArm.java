// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelIntakeArmConstants;
//import frc.robot.HardwareConfigs;

public class FuelIntakeArm extends SubsystemBase 
{
  //private SparkMax fuelIntakeArmMotor;
  //private RelativeEncoder fuelIntakeArmEncoder;
  private SparkMax fuelIntakeRightArmMotor;
  private SparkMaxConfig fuelIntakeRightArmMotorConfig;
  private SparkMaxConfig fuelIntakeLeftArmMotorConfig;
  private SparkMax fuelIntakeLeftArmMotor;
  private RelativeEncoder fuelIntakeRightArmEncoder;
  private RelativeEncoder fuelIntakeLeftArmEncoder;
  
  //private HardwareConfigs hardwareConfigs;
  public SparkClosedLoopController rightClosedLoopController;
  public SparkClosedLoopController leftClosedLoopController;
  public double currentFuelIntakeArmTarget = 0.0;

  public FuelIntakeArm() 
  {
    double degreesPerMotorRotation = 360.0 / FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_MOTOR_GEAR_RATIO;

    fuelIntakeRightArmMotor = new SparkMax(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
    fuelIntakeRightArmMotorConfig = new SparkMaxConfig();
    fuelIntakeRightArmMotorConfig.encoder.positionConversionFactor(degreesPerMotorRotation);
    fuelIntakeRightArmMotorConfig.encoder.velocityConversionFactor(degreesPerMotorRotation / 60);
    fuelIntakeRightArmMotorConfig.closedLoop.p(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_P);
    fuelIntakeRightArmMotorConfig.closedLoop.i(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_I);
    fuelIntakeRightArmMotorConfig.closedLoop.d(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_D);
    fuelIntakeRightArmMotor.configure(fuelIntakeRightArmMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    

    fuelIntakeLeftArmMotor = new SparkMax(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
    fuelIntakeLeftArmMotorConfig = new SparkMaxConfig();
    fuelIntakeLeftArmMotorConfig.encoder.positionConversionFactor(degreesPerMotorRotation);
    fuelIntakeLeftArmMotorConfig.encoder.velocityConversionFactor(degreesPerMotorRotation / 60);
    fuelIntakeLeftArmMotorConfig.closedLoop.p(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_P);
    fuelIntakeLeftArmMotorConfig.closedLoop.i(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_I);
    fuelIntakeLeftArmMotorConfig.closedLoop.d(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_D);
    fuelIntakeLeftArmMotor.configure(fuelIntakeLeftArmMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    

    fuelIntakeRightArmEncoder = fuelIntakeRightArmMotor.getEncoder();
    fuelIntakeLeftArmEncoder = fuelIntakeLeftArmMotor.getEncoder();
    //hardwareConfigs = new HardwareConfigs();
    fuelIntakeRightArmEncoder.setPosition(0);
    fuelIntakeLeftArmEncoder.setPosition(0);
    //fuelIntakeAngleMotor.configure(hardwareConfigs.coralAngleSparkConfig, SparkMax.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClosedLoopController = fuelIntakeRightArmMotor.getClosedLoopController();
    leftClosedLoopController = fuelIntakeLeftArmMotor.getClosedLoopController();
  }
  
  public void setAngle(double angle)
  {
    //double setPoint = (angle / 360.0) * FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_MOTOR_GEAR_RATIO; // Convert angle to encoder counts
    rightClosedLoopController.setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    leftClosedLoopController.setSetpoint((-1 * angle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getRightAngle() 
  {
    return fuelIntakeRightArmEncoder.getPosition();
  }

  public double getLeftAngle() 
  {
    return fuelIntakeLeftArmEncoder.getPosition();
  }
 
 //Check if this is the correct method to get the angle
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FUEL INTAKE RIGHT ARM ANGLE", fuelIntakeRightArmEncoder.getPosition());
    //SmartDashboard.putNumber("FUEL INTAKE RIGHT ARM ANGLE", ((fuelIntakeRightArmEncoder.getPosition() / 20.0) * 360.0));
    SmartDashboard.putNumber("FUEL INTAKE LEFT ARM ANGLE", fuelIntakeLeftArmEncoder.getPosition());
  }
}
