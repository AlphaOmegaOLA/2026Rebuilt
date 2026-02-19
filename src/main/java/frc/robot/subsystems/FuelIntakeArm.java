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
import frc.robot.Constants.FuelIntakeArmConstants;
//import frc.robot.HardwareConfigs;

public class FuelIntakeArm extends SubsystemBase 
{
  private SparkMax fuelIntakeArmMotor;
  private RelativeEncoder fuelIntakeArmEncoder;
  //private HardwareConfigs hardwareConfigs;
  public SparkClosedLoopController closedLoopController;
  public double currentFuelIntakeArmTarget = 0.0;

  public FuelIntakeArm() 
  {
    fuelIntakeArmMotor = new SparkMax(FuelIntakeArmConstants.FuelIntakeArm.FUEL_INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);
    fuelIntakeArmEncoder = fuelIntakeArmMotor.getEncoder();
    //hardwareConfigs = new HardwareConfigs();
    fuelIntakeArmEncoder.setPosition(0);
    //fuelIntakeAngleMotor.configure(hardwareConfigs.coralAngleSparkConfig, SparkMax.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = fuelIntakeArmMotor.getClosedLoopController();
  }
  
  public void setAngle(double angle)
  {
    closedLoopController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getAngle() 
  {
    return fuelIntakeArmEncoder.getPosition();
  }
 
 //Check if this is the correct method to get the angle
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FUEL INTAKE ANGLE", fuelIntakeArmEncoder.getPosition());
  }
}
