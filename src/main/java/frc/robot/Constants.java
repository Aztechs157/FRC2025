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
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
    }

    public class VisionConstants {
        public static final String LEFT_CAMERA_NICKNAME = "Microsoft_LifeCam_HD-3000_Left"; // TODO: find proper value
        public static final Transform3d LEFT_CAMERA_PLACEMENT = new Transform3d(
                new Translation3d(0.0, 0.0, 0.72), new Rotation3d())
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0.51, 0)))
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, 0))); // TODO: find proper
                                                                                      // value,
        // new Rotation3d(0, 0.959931, 2.61799)
        public static final String RIGHT_CAMERA_NICKNAME = "Microsoft_LifeCam_HD-3000_Right"; // TODO: find proper value
        public static final Transform3d RIGHT_CAMERA_PLACEMENT = new Transform3d(
                new Translation3d(-0.305816, -0.2276856, 0.5478018), new Rotation3d())
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0.959931, 0)))
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, 0.523599))); // TODO: find proper
                                                                                             // value,

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
        public static final int BOTTOM_LIMIT_ID = 8;
        public static final int TOP_LIMIT_ID = 9;

        static final double p = 5, i = 0.0, d = 0.5;

        // TODO: find proper values for this. sysid can help, or look here https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
        // maxAccel and maxVel are fairly self explanatory, should be in percentage of travel/s(^2 for accel). start small, increase later
        static final double maxAccel=0.03, maxVel=0.03;
        // s, g, v, and a are harder to find. these are the best candidates for sysID tuning, but manual tuning should go as follows
        // s is unclear, it is meant to represent friction in the system. it is possible to leave at 0 until sysID is solved
        // g is the voltage needed to counteract gravity. it should be as large as it can without causing any motion in the elevator
        // v is the velocity feedforward, increase approx until overshoot stops.
        // a is the final value, increase until actual motion follows setpoint at slow speed.
        // after all these are tuned for slow speeds, increase speed and use PID for any fine tuning adjustments
        static final double s=0, g=0, v=0, a=0;
        // these are standard PID values, but they are much less active in control then standard PID, the feedforward should be doing 80+% of the work. tune these last
        static final double p2 = 0, i2=0, d2= 0;

        /**
         * PID controller for the elbow, with p as {@value #p}, i as {@value #i}, and d
         * as {@value #d}.
         */
        public static final PIDController PID = new PIDController(p, i, d);
        public static ProfiledPIDController NEW_PID = new ProfiledPIDController(p2, i2, d2, new TrapezoidProfile.Constraints(maxAccel, maxAccel));
        public static ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(s, g, v, a);

        public static final double SLEW_RATE_LIMIT_UP = 1.57;
        public static final double SLEW_RATE_LIMIT_DOWN = -1.57;

        /** The motor power needed to hold the elevator in place. */
        public static final double STALL_POWER = 0.057;

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
        public static final double MAX_POSITION = 2634, MIN_POSITION = 616;

        public static final double POS_TOLERANCE = 0.02;
        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
    }

    public static class IntakeConstants {

        // IDs/Ports for the sensors/actuators. The motor uses a CAN ID, the sensor uses
        // a digital I/O
        public static final int MOTOR_ID = 22;
        public static final int SENSOR_ID = 1;

        /**
         * The speed, as a percentage of total power, to run the intake motor at for the
         * specified action.
         */
        public static final double INTAKE_MOTOR_SPEED = 0.5, PLACE_MOTOR_SPEED = 0.75, EJECT_MOTOR_SPEED = 0.75;

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
        public static final double MAX_POSITION = 0.4, MIN_POSITION = 0.748;

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
        public static final double MAX_POSITION = 0.08, MIN_POSITION = 0.545;

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
        public static final int LEFT_MOTOR_ID = 30;
        public static final int RIGHT_MOTOR_ID = 31;

        /** The velocity for the manual speed control, in percentage of motor power. */
        public static final double MOTOR_SPEED = 0.5, MANUAL_CONTROL_SPEED = 0.5;

        /**
         * The margin, as a percentage (ie 1.0 = 100%) of the travel that is off limits
         * on both the top and the bottom of the travel.
         */
        public static final double LIMIT_MARGIN = 0.05;

        /**
         * Position limits, in raw encoder units, for the left Uppies motor. In this
         * case, the raw encoder unit is rotations. This defines our virtual limits.
         */
        public static final double LEFT_MAX_POSITION = 0.3228, LEFT_MIN_POSITION = 0.7053;

        /**
         * Position limits, in raw encoder units, for the right Uppies motor. In this
         * case, the raw encoder unit is rotations. This defines our virtual limits.
         */
        public static final double RIGHT_MAX_POSITION = 0.4828, RIGHT_MIN_POSITION = 0.0653;
    }
}
