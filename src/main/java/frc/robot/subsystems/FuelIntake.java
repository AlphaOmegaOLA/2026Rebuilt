// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelIntakeConstants;
//import frc.robot.HardwareConfigs;
//import frc.robot.Robot;

/**
 * This class is controls the fuel intake motors that move the fuel in and out at different speeds.
 */
public class FuelIntake extends SubsystemBase 
{
  private SparkMax fuelIntakeMotorLeft;
  private SparkMax fuelIntakeMotorRight;

  public FuelIntake() 
  {
    fuelIntakeMotorLeft = new SparkMax(FuelIntakeConstants.FuelIntake.FUEL_INTAKE_MOTOR_LEFT_ID, MotorType.kBrushless);
    fuelIntakeMotorRight = new SparkMax(FuelIntakeConstants.FuelIntake.FUEL_INTAKE_MOTOR_RIGHT_ID, MotorType.kBrushless);
    //hardwareConfigs = new HardwareConfigs();
  }
  
  private void setMotor(double speed)
  {
    fuelIntakeMotorLeft.set(speed);
    fuelIntakeMotorRight.set(speed);
  }

  public Command slow()
  {
      return this.startEnd(() -> this.setMotor(-FuelIntakeConstants.FuelIntake.HALF_SPEED),
          () -> this.setMotor(0));
  }

  public Command fast()
  {
      return this.startEnd(() -> this.setMotor(-FuelIntakeConstants.FuelIntake.FULL_SPEED),
          () -> this.setMotor(0));
  }

  public void manual(double speed)
  {
    this.setMotor(speed * .35);
  }

  public void intake(double speed)
  {
    this.setMotor(speed * .35);
  }

  public void outtake(double speed)
  {
    this.setMotor(speed * .35);
  }
}
