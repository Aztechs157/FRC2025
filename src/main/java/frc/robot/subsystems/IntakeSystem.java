// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
  private static SparkMax motor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
  private static DigitalInput sensor = new DigitalInput(IntakeConstants.SENSOR_ID);

  /** Creates a new IntakeSystem. */
  public IntakeSystem() {
    Shuffleboard.getTab("Sensor values").addBoolean("Intake Sensor", this::hasCoral);
  }

  /**
   * Runs the intake motor at a set speed percentage.
   * @param speed - The speed percentage to run the motor at (IE, values from -1.0 to 1.0)
   */
  public void run(double speed) {
    motor.set(speed);
  }
  
  /**
   * Whether the intake sensor believes that the intake is occupied or empty.
   * @return - True if the intake is full, else False
   */
  public boolean hasCoral() {
    return sensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
