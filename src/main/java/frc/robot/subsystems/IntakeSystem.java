// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

@Logged(strategy = Strategy.OPT_OUT)
public class IntakeSystem extends SubsystemBase {
  // private static SparkMax motor = new SparkMax(IntakeConstants.MOTOR_ID,
  // MotorType.kBrushless);
  final TalonFX motor = new TalonFX(IntakeConstants.MOTOR_ID);
  private static DigitalInput sensor = new DigitalInput(IntakeConstants.SENSOR_ID);
  private boolean isBeta;
  public static IntakeConstants constants;

  /**
   * Creates a new IntakeSystem.
   * 
   * @param b
   */
  public IntakeSystem(boolean isBeta) {
    this.isBeta = isBeta;
    Shuffleboard.getTab("Sensor values").addBoolean("Intake Sensor", this::hasCoral)
        .withWidget(BuiltInWidgets.kBooleanBox);
    Shuffleboard.getTab("Sensor values").addDouble("Intake Motor Current", this::getMotorCurrent)
        .withWidget(BuiltInWidgets.kTextView);
  }

  /**
   * Runs the intake motor at a set speed percentage.
   * 
   * @param speed - The speed percentage to run the motor at (IE, values from -1.0
   *              to 1.0)
   */
  public void run(double speed) {
    motor.set(speed);
  }

  /**
   * Whether the intake sensor believes that the intake is occupied or empty.
   * 
   * @return - True if the intake is full, else False
   */
  public boolean hasCoral() {
    return !sensor.get();
  }

  public double getMotorCurrent() {
    return motor.getStatorCurrent().getValue().in(Units.Amps);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // hopefully will remove the pattern if the coral sensor drops
    if(!hasCoral()){
      RobotContainer.prettyLights.removeMidPattern("Has Coral");
    }
  }
}
