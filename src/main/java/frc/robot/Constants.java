package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class ModifierConstants {
        // When true, reduces drive speed by 50%.
        public static final boolean ROOKIE_MODE = false;
        // When true, reduces drive speed by 75% and disables auto positioning.
        // Overrides ROOKIE_MODE.
        public static final boolean DEMO_MODE = false;
    }

    public static class ControllerConstants {

        // Ports for the Joysticks, as set in Driver Station
        public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.05;
        public static final double LEFT_Y_DEADBAND = 0.05;
        public static final double RIGHT_X_DEADBAND = 0.05;

        public static final double PRECISION_DRIVE_MODIFIER = 0.75;
    }

    public class VisionConstants {
        public static final String TOPRIGHT_CAMERA_NICKNAME = "Microsoft_LifeCam_HD-3000_TopRight";
        public static final Transform3d TOPRIGHT_CAMERA_PLACEMENT = new Transform3d(
                new Translation3d(0.203414, -0.196768, 0.958612), new Rotation3d(0, -0.436332, 0));
        public static final String BOTTOM_CAMERA_NICKNAME = "Microsoft_LifeCam_HD-3000_Bottom";
        public static final Transform3d BOTTOM_CAMERA_PLACEMENT = new Transform3d(
                new Translation3d(0.2602992, 0, 0.126), new Rotation3d(0, -0.349066, 0)); // new Translation3d(0.5653,
                                                                                          // 0, 0.120), new
                                                                                          // Rotation3d(0, 0.35 , 0)
                                                                                          // Gets us really close to
                                                                                          // correct

        public static final PIDController AIMING_PID = new PIDController(0.05, 0, 0.01);
        // How close the robot can be (bumper to tag, in meters) before losing the
        // ability to auto-align.
        public static final double MIN_DISTANCE_TO_TAG = 0.8;
    }

    public static class LoggingConstants {

        public static final boolean DEFAULT_LOGGING_STATE = false;
    }

    public static class ElevatorConstants {

        // IDs/Ports for the sensors/actuators. The motor uses a CAN ID, the POT (short
        // for potentiometer) uses an analog input port, and the limits use digital I/O
        // ports
        public static final int MOTOR_ID = 21;
        public static final int POT_ID = 0;
        public static final int BOTTOM_LIMIT_ID = 8, TOP_LIMIT_ID = 9;

        // Gear ratio of the elevator's gearbox.
        public static final double GEAR_RATIO = 15.0;
        // Mass of the elevator's carriage, in kilograms. 
        public static final double CARRIAGE_MASS = Units.lbsToKilograms(23);
        // Pitch radius of the elevator's sprocket, in meters.
        public static final double DRUM_RADIUS = Units.inchesToMeters(1.757)/2;
        // Maxmimum height of the elevator's carriage, in meters.
        public static final double MAX_CARRIAGE_HEIGHT = 1.3285;

        static final double p = 5, i = 0.0, d = 0.5;

        // TODO: find proper values for this. sysid can help, or look here
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
        // maxAccel and maxVel are fairly self explanatory, should be in percentage of
        // travel/s(^2 for accel). start small, increase later
        static final double alphaMaxAccel = 1, alphaMaxVel = 0.7;
        static final double betaMaxAccel = 1.75, betaMaxVel = 1;
        // s, g, v, and a are harder to find. these are the best candidates for sysID
        // tuning, but manual tuning should go as follows
        // s is unclear, it is meant to represent friction in the system. it is possible
        // to leave at 0 until sysID is solved
        // g is the voltage needed to counteract gravity. it should be as large as it
        // can without causing any motion in the elevator
        // v is the velocity feedforward, increase approx until overshoot stops.
        // a is the final value, increase until actual motion follows setpoint at slow
        // speed.
        // after all these are tuned for slow speeds, increase speed and use PID for any
        // fine tuning adjustments s = 0, g = 0.5, v = 10, a = 0.7;
        static final double as = 0, ag = 0.9, av = 9, aa = 0.7;
        // bg could go lower but is fine for now.
        static final double bs = 0.35921, bg = 0.79494, bv = 8.1184, ba = 0.93;
        // v = 8.6, a = 0.05
        // these are standard PID values, but they are much less active in control then
        // standard PID, the feedforward should be doing 80+% of the work. tune these
        // last
        static final double ap = 1, ai = 0, ad = 0;

        static final double bp = 25, bi = 0, bd = 0;

        /**
         * PID controller for the elbow, with p as {@value #p}, i as {@value #i}, and d
         * as {@value #d}.
         */

        public static ProfiledPIDController ALPHA_NEW_PID = new ProfiledPIDController(ap, ai, ad,
                new TrapezoidProfile.Constraints(alphaMaxAccel, alphaMaxAccel));
        public static ElevatorFeedforward ALPHA_FEEDFORWARD = new ElevatorFeedforward(as, ag, av, aa);

        public static ProfiledPIDController BETA_NEW_PID = new ProfiledPIDController(bp, bi, bd,
                new TrapezoidProfile.Constraints(betaMaxAccel, betaMaxAccel));
        public static ElevatorFeedforward BETA_FEEDFORWARD = new ElevatorFeedforward(bs, bg, bv, ba);

        public static final double SLEW_RATE_LIMIT_UP = 1.57;
        public static final double SLEW_RATE_LIMIT_DOWN = -1.57;

        /** The motor power needed to hold the elevator in place. */
        public static final double ALPHA_STALL_POWER = 0.057;
        public static final double BETA_STALL_POWER = 0.045;

        /**
         * The power used to move the motor for manual control, ie not in "go to
         * position" mode.
         */
        public static final double MANUAL_CONTROL_SPEED_UP = 0.25, MANUAL_CONTROL_SPEED_DOWN = 0.157;

        /**
         * The margin, as a percentage (ie 1.0 = 100%) of the travel that is off limits
         * on both the top and the bottom of the travel.
         */
        public static final double LIMIT_MARGIN = 0.01;

        // max is top-most for all systems, min is bottom-most for all systems
        /**
         * Position limits, in raw encoder units.
         * In this case, rotations. This defines
         * our virtual limits.
         */
        public static final double ALPHA_MAX_POSITION = 2634, ALPHA_MIN_POSITION = 616;
        public static final double BETA_MAX_POSITION = 3100, BETA_MIN_POSITION = 1115;

        public static final double POS_TOLERANCE = 0.01;
        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
    }

    public static class IntakeConstants {

        // IDs/Ports for the sensors/actuators. The motor uses a CAN ID, the sensor uses
        // a digital I/O
        public static final int MOTOR_ID = 22;
        public static final int SENSOR_ID = 1;
        public static final double INTAKE_ALGAE_SPEED = 1, ALGAE_HOLDING_SPEED = 0.1, CORAL_HOLDING_SPEED = 0.02;
        public static final double ALGAE_HELD_CURRENT = 40, CORAL_HELD_CURRENT = 55;
        public static final int ALGAE_HOLD_COUNTER = 10, CORAL_HOLD_COUNTER = 5;

        /**
         * The speed, as a percentage of total power, to run the intake motor at for the
         * specified action.
         */
        public static final double INTAKE_MOTOR_SPEED = 0.75, PLACE_MOTOR_SPEED = 1, EJECT_MOTOR_SPEED = 1;

        /** The time to wait before considering the action done, in seconds. */
        public static final double PLACE_TIME = 0.5, EJECT_TIME = 0.5;
    }

    public static class ElbowConstants {
        // IDs/Ports for the sensors/actuators. The motor uses a CAN ID, the encoder
        // uses a digital I/O
        public static final int MOTOR_ID = 23;
        public static final int ENCODER_ID = 0;

        static final double p = 2.0, i = 0.0, d = 0.0001;

        /**
         * PID controller for the elbow, with p as {@value #p}, i as {@value #i}, and d
         * as {@value #d}.
         */
        public static final PIDController PID = new PIDController(p, i, d);

        /**
         * Position limits, in raw encoder units. In this case, rotations. This defines
         * our virtual limits.
         */
        public static final double ALPHA_MAX_POSITION = 0.4, ALPHA_MIN_POSITION = 0.748;
        public static final double BETA_MAX_POSITION = 0.82, BETA_MIN_POSITION = 0.418;

        /** The velocity for the manual speed control, in percentage of motor power. */
        public static final double MANUAL_CONTROL_SPEED = 0.25;

        /**
         * The margin, as a percentage (ie 1.0 = 100%) of the travel that is off limits
         * on both the top and the bottom of the travel.
         */
        public static final double LIMIT_MARGIN = 0.01;

        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
        public static final double POS_TOLERANCE = 0.02;
    }

    public static class WristConstants {
        // IDs/Ports for the sensors/actuators. The motor uses a CAN ID
        public static final int MOTOR_ID = 24;

        static final double p = 3, i = 0.0, d = 0.0001;

        /**
         * PID controller for the wrist, with p as {@value #p}, i as {@value #i}, and d
         * as {@value #d}.
         */
        public static final PIDController PID = new PIDController(p, i, d);

        /**
         * Position limits, in raw encoder units. In this case, rotations. This defines
         * our virtual limits. The minimum position is when the elbow is at the top.
         */
        public static final double ALPHA_MAX_POSITION = 0.08, ALPHA_MIN_POSITION = 0.545;
        // Original values: BETA_MAX = 0.827, BETA_MIN = 0.271
        public static final double BETA_MAX_POSITION = 0.83, BETA_MIN_POSITION = 0.274;

        /** The velocity for the manual speed control, in percentage of motor power. */
        public static final double MANUAL_CONTROL_SPEED = 0.25;

        /**
         * The margin, as a percentage (ie 1.0 = 100%) of the travel that is off limits
         * on both the top and the bottom of the travel.
         */
        public static final double LIMIT_MARGIN = 0.01;

        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
        public static final double POS_TOLERANCE = 0.02;
    }

    public static class UppiesConstants {
        // IDs/Ports for the sensors/actuators. The motors uses a CAN ID
        public static final int MOTOR_ID = 30;
        public static final int MOTOR_ID_FOLLOWER = 31;
        public static final int ENCODER_ID = 2;

        /** The velocity for the manual speed control, in percentage of motor power. */
        public static final double MOTOR_SPEED = 0.75, MANUAL_CONTROL_SPEED = 1, STALL_SPEED = 0.25;
        public static final double MOTOR_DESYNC_RATIO = 0.9; // to be applied to the left motor to sync it up with
                                                             // the
                                                             // right

        /**
         * The margin, as a percentage (ie 1.0 = 100%) of the travel that is off limits
         * on both the top and the bottom of the travel.
         */
        public static final double LIMIT_MARGIN = 0.00;

        /**
         * Position limits, in raw encoder units, for the left Uppies motor. In this
         * case, the raw encoder unit is rotations. This defines our virtual limits.
         */
        public static final double MAX_POSITION = 0.635, MIN_POSITION = 0.373;
    }

    public static class LEDConstants {
        // The PMW port the strip is plugged into on the RIO.
        public static final int PMW_PORT = 9;
        // The amount of diodes per section of the strip (not the total amount).
        public static final int STRIP_LENGTH = 18;
        // Distance between each diode in millimeters (60led/m strip)
        public static final double DISTANCE_PER_LED = 15;
        // The voltage at which the battery warning lights will flash.
        public static final double BATTERY_WARNING_VOLTAGE = 11.5;
    }

    public static class ModelConstants {
        // Offsets of each major subsytem, in meters, relative to 
        // robot center (floor), for use with the AdvantageScope model.
        public static final Pose3d ELEVATOR_OFFSET = new Pose3d(0.1905, 0.0, 0.26, new Rotation3d());
        public static final Pose3d ELBOW_OFFSET = new Pose3d(0.26, 0.0, 0.29, new Rotation3d());
        public static final Pose3d UPPIES_OFFSET = new Pose3d(-0.28, 0.0, 0.38, new Rotation3d());
        /** Maximum height, in meters, for each component of the elevator. */
        public static final double CARRIAGE_MAX_HEIGHT = 1.3285, STAGE_TWO_MAX_HEIGHT = 0.665;
        /** Angle limits, in radians, of the elbow. */
        public static final double ELBOW_MAX_ANGLE = -1.43, ELBOW_MIN_ANGLE = 1.0;
        /** Angle limits, in radians, of the climber. */
        public static final double UPPIES_MIN_ANGLE = -0.2, UPPIES_MAX_ANGLE = -2;
        /** Angle limits, in radians, of the wrist. */
        public static final double WRIST_MIN_ANGLE = 2.6, WRIST_MAX_ANGLE = -0.7;        
    }

}
