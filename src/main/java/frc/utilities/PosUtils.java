// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utilities;

/** Add your docs here. */
public interface PosUtils {

    /**
     * checks if the currentPos is within posTolerance of desiredPos and the motorVelocity is within velocityTolerance in the direction towards desiredPos.
     * @param desiredPos - the position you wish the system to reach
     * @param currentPos - the position the system is currently at
     * @param posTolerance - the allowable deviation from the desired position
     * @param motorVelocity - the velocity that the motor is currently spinning at
     * @param velocityTolerance - the maximum motor velocity for the system to not overshoot the desired position tolerance
     * @return - boolean
     */
    static boolean isOscillating(double desiredPos, double currentPos, double posTolerance, double motorVelocity, double velocityTolerance) {
        boolean retval = false;
    
        if ((currentPos >= desiredPos - posTolerance) & ((motorVelocity >= 0) & (motorVelocity <= velocityTolerance))) {
          // checks if the elevator's current position is below the desired position within ELEVATOR_POS_TOLERANCE and the elevator is moving up at a speed below ELEVATOR_MOTOR_VELOCITY_TOLERANCE
          retval = true;
        } 
        else if ((currentPos <= desiredPos + posTolerance) & ((motorVelocity <= 0) & (motorVelocity >= - velocityTolerance))) {
          // checks if the elevator's current position is above the desired position within ELEVATOR_POS_TOLERANCE and the elevator is moving down at a speed below ELEVATOR_MOTOR_VELOCITY_TOLERANCE
          retval = true;
        }
    
        return retval;
    }
    
}
