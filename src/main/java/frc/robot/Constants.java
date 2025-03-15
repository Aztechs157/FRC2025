package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
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
        public static final String TOPRIGHT_CAMERA_NICKNAME = "Microsoft_LifeCam_HD-3000_TopRight"; // TODO: find proper
                                                                                                    // value
        public static final Transform3d TOPRIGHT_CAMERA_PLACEMENT = new Transform3d(
                new Translation3d(0.229, 0.165, 1.003), new Rotation3d())
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0.44, 0))) // This line to set possible
                                                                                        // pitch
                .plus(new Transform3d(new Translation3d(), new Rotation3d(Math.PI / 2, 0, 0))); // This line to set
                                                                                                // possible
        // yaw,
        // new Rotation3d(0, 0.959931, 2.61799)
        public static final String BOTTOM_CAMERA_NICKNAME = "Microsoft_LifeCam_HD-3000_Bottom"; // TODO: find proper
                                                                                                // value
        public static final Transform3d BOTTOM_CAMERA_PLACEMENT = new Transform3d(
                new Translation3d(0.254, 0, 0.178), new Rotation3d())
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0.35, 0))) // This line to set possible
                                                                                        // pitch
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, 0))); // This line to set possible yaw,

        public static final PIDController AIMING_PID = new PIDController(0.05, 0, 0.01);
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
        public static final double LIMIT_MARGIN = 0.05;

        // max is top-most for all systems, min is bottom-most for all systems
        /**
         * Position limits, in raw encoder units. In this case, rotations. This defines
         * our virtual limits.
         */
        public static final double ALPHA_MAX_POSITION = 2634, ALPHA_MIN_POSITION = 616;
        public static final double BETA_MAX_POSITION = 4095, BETA_MIN_POSITION = 2080;

        public static final double POS_TOLERANCE = 0.01;
        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
    }

    public static class IntakeConstants {

        // IDs/Ports for the sensors/actuators. The motor uses a CAN ID, the sensor uses
        // a digital I/O
        public static final int MOTOR_ID = 22;
        public static final int SENSOR_ID = 1;
        public static final double INTAKE_ALGAE_SPEED = 1, ALGAE_HOLDING_SPEED = 0.1, CORAL_HOLDING_SPEED = 0.01;
        public static final double ALGAE_HELD_CURRENT = 40, CORAL_HELD_CURRENT = 40;
        public static final int ALGAE_HOLD_COUNTER = 25, CORAL_HOLD_COUNTER = 5;

        /**
         * The speed, as a percentage of total power, to run the intake motor at for the
         * specified action.
         */
        public static final double INTAKE_MOTOR_SPEED = 0.357, PLACE_MOTOR_SPEED = 0.5, EJECT_MOTOR_SPEED = 0.5;

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
        public static final double BETA_MAX_POSITION = 0.825, BETA_MIN_POSITION = 0.444;

        /** The velocity for the manual speed control, in percentage of motor power. */
        public static final double MANUAL_CONTROL_SPEED = 0.25;

        /**
         * The margin, as a percentage (ie 1.0 = 100%) of the travel that is off limits
         * on both the top and the bottom of the travel.
         */
        public static final double LIMIT_MARGIN = 0.05;

        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
        public static final double POS_TOLERANCE = 0.02;
    }

    public static class WristConstants {
        // IDs/Ports for the sensors/actuators. The motor uses a CAN ID
        public static final int MOTOR_ID = 24;

        static final double p = 1.0, i = 0.0, d = 0.0001;

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
        public static final double BETA_MAX_POSITION = 0.827, BETA_MIN_POSITION = 0.271;

        /** The velocity for the manual speed control, in percentage of motor power. */
        public static final double MANUAL_CONTROL_SPEED = 0.25;

        /**
         * The margin, as a percentage (ie 1.0 = 100%) of the travel that is off limits
         * on both the top and the bottom of the travel.
         */
        public static final double LIMIT_MARGIN = 0.05;

        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
        public static final double POS_TOLERANCE = 0.02;
    }

    public static class UppiesConstants {
        // IDs/Ports for the sensors/actuators. The motors uses a CAN ID
        public static final int MOTOR_ID = 30;
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
        public static final double LIMIT_MARGIN = 0.05;

        /**
         * Position limits, in raw encoder units, for the left Uppies motor. In this
         * case, the raw encoder unit is rotations. This defines our virtual limits.
         */
        public static final double MAX_POSITION = 0.486, MIN_POSITION = 0.185;
    }
}
