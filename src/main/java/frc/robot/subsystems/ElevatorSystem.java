// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utilities.PosUtils;

public class ElevatorSystem extends SubsystemBase implements PosUtils {

  private static SparkMax motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private static AnalogInput pot = new AnalogInput(ElevatorConstants.ELEVATOR_POT_ID);
  private static DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.ELEVATOR_BOTTOM_LIMIT_ID);
  private static DigitalInput topLimit = new DigitalInput(ElevatorConstants.ELEVATOR_TOP_LIMIT_ID);
  private static PIDController PID = ElevatorConstants.ELEVATOR_PID;

  /** Creates a new ElevatorSystem. */
  public ElevatorSystem() {
    Shuffleboard.getTab("Sensor values").addDouble("Elevator Pot", this::getPos);
    Shuffleboard.getTab("Sensor values").addBoolean("Elevator Bottom Limit Switch", this::atBottom);
    Shuffleboard.getTab("Sensor values").addBoolean("Elevator Top Limit Switch", this::atTop);
  }

  public void runMotor(double velocity) {
    motor.set(velocity);
  }

  public double getMotorVelocity() {
    return motor.getEncoder().getVelocity();
  }

  public double getPos() {
    return pot.getValue();
  }

  public boolean atLimit(boolean top) {
    return top? atTop() : atBottom();
  }

  public boolean atBottom() {
    return bottomLimit.get();
  }

  public boolean atTop() {
    return topLimit.get();
  }

  public double getNewSpeed(double desiredPos) {
    return PID.calculate(getPos(), desiredPos);
  }

  public boolean isOscillating(double desiredPos) {
    return PosUtils.isOscillating(desiredPos, getPos(), ElevatorConstants.ELEVATOR_POS_TOLERANCE, getMotorVelocity(), ElevatorConstants.ELEVATOR_MOTOR_VELOCITY_TOLERANCE);
  }

  public Command elevatorManualControl(Double joystickInput) {
    System.out.println("asdfsadf");
    return runEnd(() -> {runMotor(joystickInput);}, () -> {runMotor(0);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
