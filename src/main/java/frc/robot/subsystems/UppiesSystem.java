// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UppiesConstants;

public class UppiesSystem extends SubsystemBase {
  private static SparkMax motorLeft = new SparkMax(UppiesConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static SparkMax motorRight = new SparkMax(UppiesConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new uppiesSystem. */
  public UppiesSystem() {
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Left Pos", this::getPosLeft);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Right Pos", this::getPosRight);
  }

  /**
   * Runs the uppies motor at a set speed percentage 
   * @param speed - The speed percentage to run the motor at (IE, values from -1.0 to 1.0)
   */
  public void run(double speed) {
    motorLeft.set(speed);
    motorRight.set(-speed);
  }
  
  /**
   * Gets the uppies pos 
   * @return - double encoder pos
   */
  private double getPosLeft() {
    return motorLeft.getAbsoluteEncoder().getPosition();
  }

  private double getPosRight() {
    return motorRight.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
