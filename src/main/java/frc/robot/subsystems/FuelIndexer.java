// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelIndexerConstants;;
//import frc.robot.HardwareConfigs;
//import frc.robot.Robot;

/**
 * This class is controls the fuel intake motors that move the fuel in and out at different speeds.
 */
public class FuelIndexer extends SubsystemBase 
{
  private SparkMax fuelIndexerMotor;

  public FuelIndexer() 
  {
    fuelIndexerMotor = new SparkMax(FuelIndexerConstants.FuelIndexer.FUEL_INDEXER_MOTOR_ID, MotorType.kBrushless);
    //hardwareConfigs = new HardwareConfigs();
  }
  
  private void setMotor(double speed)
  {
    fuelIndexerMotor.set(speed);
  }

  public Command slow()
  {
      return this.startEnd(() -> this.setMotor(-FuelIndexerConstants.FuelIndexer.HALF_SPEED),
          () -> this.setMotor(0));
  }

  public Command fast()
  {
      return this.startEnd(() -> this.setMotor(-FuelIndexerConstants.FuelIndexer.FULL_SPEED),
          () -> this.setMotor(0));
  }

  public void manual(double speed)
  {
    this.setMotor(speed * .35);
  }
}
