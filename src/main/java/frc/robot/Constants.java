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

        public static final int MOTOR_ID = 21;
        public static final int POT_ID = 0;
        public static final int BOTTOM_LIMIT_ID = 8;
        public static final int TOP_LIMIT_ID = 9;
        
        public static final PIDController PID = new PIDController(0, 0, 0);
        public static final double STALL_POWER = 0.057;
        public static final double MANUAL_CONTROL_SPEED_UP = 0.25;
        public static final double MANUAL_CONTROL_SPEED_DOWN = 0.157;
        public static final double LIMIT_MARGIN = 0.05;
        public static final double MAX_POSITION = 2650.0; // max is top-most for all systems
        public static final double MIN_POSITION = 625.0; // min is bottom-most for all systems
        
        public static final double POS_TOLERANCE = 2;
        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
    }

    public static class IntakeConstants {

        public static final int MOTOR_ID = 22;
        public static final int SENSOR_ID = 1;

        public static final double INTAKE_MOTOR_SPEED = 0.5;
        public static final double PLACE_MOTOR_SPEED = 0.75;
        public static final double EJECT_MOTOR_SPEED = 0.75;

        public static final double PLACE_TIME = 0.5;
        public static final double EJECT_TIME = 0.5;
    }

    public static class ElbowConstants {
        public static final int MOTOR_ID = 23;
        public static final int ENCODER_ID = 0;

        public static final PIDController PID = new PIDController(0, 0, 0);
        public static final double MAX_POSITION = 0.4;
        public static final double MIN_POSITION = 0.748; // 0.726 when elevator at bottom
        public static final double MANUAL_CONTROL_SPEED = 0.25;
        public static final double LIMIT_MARGIN = 0.05;


        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
        public static final double POS_TOLERANCE = 2;
    }

    public static class WristConstants {
        public static final int MOTOR_ID = 24;

        public static final PIDController PID = new PIDController(0, 0, 0);
        public static final double MAX_POSITION = 0.08; // 0.545 when elbow at top
        public static final double MIN_POSITION = 0.545;
        public static final double MANUAL_CONTROL_SPEED = 0.25;
        public static final double LIMIT_MARGIN = 0.05;

        public static final double MOTOR_VELOCITY_TOLERANCE = 0.2;
        public static final double POS_TOLERANCE = 2;
    }

    public static class UppiesConstants {
        public static final int LEFT_MOTOR_ID = 30;
        public static final int RIGHT_MOTOR_ID = 31;

        public static final double MOTOR_SPEED = 0.5;
        public static final double MANUAL_CONTROL_SPEED = 0.5;
        public static final double LIMIT_MARGIN = 0.05;

        public static final double LEFT_MAX_POSITION = 0.3228;
        public static final double LEFT_MIN_POSITION = 0.7053;

        public static final double RIGHT_MAX_POSITION = 0.4828;
        public static final double RIGHT_MIN_POSITION = 0.0653;
    }
}

