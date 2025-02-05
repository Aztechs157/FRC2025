package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
    public static class ControllerConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
    }

    public class VisionConstants
    {

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

        public static final int ELEVATOR_MOTOR_ID = 21;
        public static final int ELEVATOR_POT_ID = 0;
        public static final int ELEVATOR_BOTTOM_LIMIT_ID = 8;
        public static final int ELEVATOR_TOP_LIMIT_ID = 9;

        public static final PIDController ELEVATOR_PID = new PIDController(0, 0, 0);

        public static final double ELEVATOR_POS_TOLERANCE = 2;
        public static final double ELEVATOR_MOTOR_VELOCITY_TOLERANCE = 0.2;
    }

    public static class IntakeConstants {

        public static final int INTAKE_MOTOR_ID = 22;
        public static final int INTAKE_SENSOR_ID = 1;
        // public static final int INTAKE_SENSOR2_ID = 33;

        public static final double INTAKE_MOTOR_SPEED = 0.5;
        public static final double PLACE_MOTOR_SPEED = 0.5;
        public static final double EJECT_MOTOR_SPEED = 0.5;

        public static final double PLACE_TIME = 0.5;
        public static final double EJECT_TIME = 0.5;
    }

    public static class ElbowConstants {
        public static final int ELBOW_MOTOR_ID = 23;
        public static final int ELBOW_ENCODER_ID = 0;

        public static final PIDController ELBOW_PID = new PIDController(0, 0, 0);
    }

    public static class WristConstants {
        public static final int WRIST_MOTOR_ID = 24;

        public static final PIDController WRIST_PID = new PIDController(0, 0, 0);

        public static final double WRIST_MANUAL_CONTROL_SPEED = 0.25;
    }
}

