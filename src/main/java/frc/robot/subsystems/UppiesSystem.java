// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value; // TODO: what is this!!!???

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.UppiesConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;
import frc.utilities.PosUtils;

@Logged(strategy = Strategy.OPT_OUT)
public class UppiesSystem extends SubsystemBase {
  private static TalonFX motor = new TalonFX(UppiesConstants.MOTOR_ID);
  private static TalonFX motor_follower = new TalonFX(UppiesConstants.MOTOR_ID_FOLLOWER);
  private static DutyCycleEncoder encoderInput = new DutyCycleEncoder(UppiesConstants.ENCODER_ID);

  /**
   * Creates a new UppiesSystem.
   * 
   * @param b
   */
  public UppiesSystem(boolean b) {
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Pos", this::getPos)
        .withWidget(BuiltInWidgets.kTextView);
    Shuffleboard.getTab("Sensor values").addDouble("Uppies Scaled Pos", this::getScaledPos)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "center", 0, "max", 1));
  }

  /**
   * Runs the left motor at a set velocity.
   * <p>
   * <b>WARNING: this ignores limits entirely and can damage the robot, consider
   * using another method.</b>
   * </p>
   * 
   * @param velocity Velocity is actually a percentage of speed, from -1.0 to 1.0
   */
  public void runMotor(double velocity) {
    motor.set(velocity);
    motor_follower.set(velocity);
  }

  /**
   * Tells the Uppies system to run the motor at a set velocity. Properly follows
   * position limits defined by
   * {@link frc.robot.Constants.UppiesConstants#LIMIT_MARGIN},
   * such that the motors are only not allowed to move past a scaled position of
   * {@value frc.robot.Constants.ElevatorConstants#LIMIT_MARGIN} for the bottom
   * limit, or
   * 1.0-{@value frc.robot.Constants.ElevatorConstants#LIMIT_MARGIN} for the top
   * limit.
   * 
   * @param speed The speed to move the motor at, between -1.0 to 1.0 (ie, 1.0
   *              moves the Uppies motors with the intent to [TODO check if 1.0
   *              moves the robot up or down wrt climbing])
   * @see frc.robot.subsystems.UppiesSystem#getScaledPosLeft()
   */
  public double runWithLimits(double speed) {
    if (getScaledPos() >= 1.0 - UppiesConstants.LIMIT_MARGIN && speed > 0) {
      return 0;
    } else if (getScaledPos() >= 0.9 - UppiesConstants.LIMIT_MARGIN && speed > 0) {
      return speed * 0.75;
    } else if (getScaledPos() <= 0.0 + UppiesConstants.LIMIT_MARGIN && speed < 0) {
      return 0;
    } else if (getScaledPos() <= 0.1 + UppiesConstants.LIMIT_MARGIN && speed < 0) {
      return speed * 0.75;
    } else {
      return speed;
    }
  }

  /**
   * Gets the position of the left Uppies motor absolute encoder.
   * 
   * @return - the encoder value, in rotations (from 0.0 to 1.0)
   */
  public double getPos() {
    return encoderInput.get();
  }

  /**
   * Gets the position of the right Uppies motor absolute encoder, after scaling
   * to the possible range of motion.
   * 
   * @return - the encoder value, in percentage of travel distance (from 0.0 to
   *         1.0)
   */
  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), UppiesConstants.MIN_POSITION, UppiesConstants.MAX_POSITION, 0.0,
        1.0);
  }

  /**
   * Gets the current angle of Uppies for AdvantageScope model use.
   * 
   * @return - the uppies encoder value, scaled to radians
   */
  public double getScaledPosAngle() {
    return PosUtils.mapRange(getPos(), UppiesConstants.MIN_POSITION, UppiesConstants.MAX_POSITION, -0.2, 
    -2);
  }

  public boolean atPosition(double position) {
    boolean reached = false;
    if ((getScaledPos() - UppiesConstants.LIMIT_MARGIN) <= position) {
      reached = true;
    }

    return reached;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getScaledPos() >= 1.0 - UppiesConstants.LIMIT_MARGIN) {
      LEDPattern uppy = LEDPattern.solid(Color.kWhiteSmoke);
      RobotContainer.prettyLights.addMidPattern("Uppies Deployed", 7, uppy);
    } else {
      RobotContainer.prettyLights.removeMidPattern("Uppies Deployed");
    }
  }
}
