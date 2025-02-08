// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UppiesConstants;
import frc.utilities.PosUtils;

public class UppiesSystem extends SubsystemBase {
  private static SparkMax motorLeft = new SparkMax(UppiesConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static SparkMax motorRight = new SparkMax(UppiesConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new uppiesSystem. */
  public UppiesSystem() {
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Left Pos", this::getPosLeft);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Right Pos", this::getPosRight);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Scaled Left Pos", this::getScaledPosLeft);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Scaled Right Pos", this::getScaledPosRight);
  }

  /** TODO: update this
   * Runs the uppies motor at a set speed percentage
   * @param speed - The speed percentage to run the motor at (IE, values from -1.0 to 1.0)
   */
  public void run(double speed) {
    if (getScaledPosLeft() >= 0.8 && speed < 0) {
      motorLeft.set(0);
    } else if (getScaledPosLeft() >= 0.7 && speed < 0) {
      motorLeft.set(speed * 0.75);
    } else if (getScaledPosLeft() <= 0.2 && speed > 0) {
      motorLeft.set(0);
    } else if (getScaledPosLeft() <= 0.3 && speed > 0) {
      motorLeft.set(speed * 0.75);
    } else {
      motorLeft.set(speed);
    }

    if (getScaledPosRight() >= 0.8 && speed < 0) {
      motorRight.set(0);
    } else if (getScaledPosRight() >= 0.7 && speed < 0) {
      motorRight.set(-speed * 0.75);
    } else if (getScaledPosRight() <= 0.2 && speed > 0) {
      motorRight.set(0);
    } else if (getScaledPosRight() <= 0.3 && speed > 0) {
      motorRight.set(-speed * 0.75);
    } else {
      motorRight.set(-speed);
    }
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

  public double getScaledPosLeft(){
    return PosUtils.mapRange(getPosLeft(), UppiesConstants.LEFT_MIN_POSITION, UppiesConstants.LEFT_MAX_POSITION, 0.0, 1.0);
  }

  public double getScaledPosRight(){
    return PosUtils.mapRange(getPosRight(), UppiesConstants.RIGHT_MIN_POSITION, UppiesConstants.RIGHT_MAX_POSITION, 0.0, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
