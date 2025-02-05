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
  private static SparkMax motor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private static DigitalInput sensor = new DigitalInput(IntakeConstants.INTAKE_SENSOR_ID);

  /** Creates a new IntakeSystem. */
  public IntakeSystem() {
    Shuffleboard.getTab("Sensor values").addBoolean("Intake Sensor", this::hasCoral);
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public boolean hasCoral() {
    return sensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
