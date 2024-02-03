package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class Swerve {
        /*
        stores the robot specific constants {
            frontRightOffset [0],
            frontLeftOffset [1],
            backLeftOffset [2],
            backRightOffset [3],
            drivingGearRatio [4],
            maxSpeedFeetPerSec [5],
            maxAngularSpeedFeetPerSec [6],
            wheelCircumferenceInches [7],
            turningKPval [8],
            profiledKPval [9],
            maxVelocityMetersPerSec,
            wheelbaseInches [11],
            trackwidthInches [12],
            maxAccelerationMetersPerSecond [13],
            turningGearRatio [14], 
            isMK4i  (1 means it is MK4i, 0 means its not) [15]
        }
         */
        public static final double[] GOLDMODULE_CONSTANTS = { 338, 107, 311, 29, 5.70, 15.82, 15.82, 4 * Math.PI, 0.015,
                1, 6.35, 21.5, 24.5, Math.PI, 150.0 / 7, 1 };
        public static final double[] QB_CONSTANTS = { 351, 229, 169, 207, 8.14, 10.9, 10.9, 4 * Math.PI, 0.015, 
                1, 6.92, 19, 19, Math.PI, 150.0 / 7, 1 };
        public static final double[] LITEBOT_CONSTANTS = { 194, -5, 2, 268, 6.12, 14.73, 14.73, 4 * Math.PI, 0.0000001, 
                1, 4.49, 21, 24, Math.PI, 12.8, 0 };

        public static final Map<String, double[]> ROBOT_MAP = new HashMap<String, double[]>() {
            {
                put("GOLDMODULE", GOLDMODULE_CONSTANTS);
                put("QB", QB_CONSTANTS);
                put("LITEBOT", LITEBOT_CONSTANTS);
            }
        };

        // CHOOSE WHICH ROBOT YOU'RE USING
        public static final double[] CHOSEN_CONSTANTS = ROBOT_MAP.get("LITEBOT");

        // miscellaneous constants
        public static final double MAX_SPEED_METERS_PER_SECONDS = Units.feetToMeters(CHOSEN_CONSTANTS[5]);
        public static final double MAX_ANGULAR_SPEED_METERS_PER_SECOND = Units.feetToMeters(CHOSEN_CONSTANTS[6]);
        public static final double TURNING_GEAR_RATIO = CHOSEN_CONSTANTS[14];
        public static final double DRIVING_GEAR_RATIO = CHOSEN_CONSTANTS[4];
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(CHOSEN_CONSTANTS[7]);
        public static final double THETA_KP_VALUE = CHOSEN_CONSTANTS[8];
        public static final double PROFILED_KP_VALUE = CHOSEN_CONSTANTS[9];
        public static final double MAX_VELOCITY_RADIANS_PER_SECOND = CHOSEN_CONSTANTS[10];
        public static final double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = CHOSEN_CONSTANTS[13];
        public static final double DISCRETIZE_TIMESTAMP = 0.02;
        public static final double isMK4i = CHOSEN_CONSTANTS[15];

        // front right wheel
        public static final int FRONT_RIGHT_DRIVE_ID = 13;
        public static final int FRONT_RIGHT_TURN_ID = 12;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = CHOSEN_CONSTANTS[0];

        // front left wheel
        public static final int FRONT_LEFT_DRIVE_ID = 23;
        public static final int FRONT_LEFT_TURN_ID = 22;
        public static final int FRONT_LEFT_CANCODER_ID = 21;
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = CHOSEN_CONSTANTS[1];

        // back left wheel
        public static final int BACK_LEFT_DRIVE_ID = 33;
        public static final int BACK_LEFT_TURN_ID = 32;
        public static final int BACK_LEFT_CANCODER_ID = 31;
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET = CHOSEN_CONSTANTS[2];

        // back right wheel
        public static final int BACK_RIGHT_DRIVE_ID = 43;
        public static final int BACK_RIGHT_TURN_ID = 42;
        public static final int BACK_RIGHT_CANCODER_ID = 41;
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = CHOSEN_CONSTANTS[3];

        public static final TrapezoidProfile.Constraints TRAPEZOID_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double TRACK_WIDTH = Units.inchesToMeters(CHOSEN_CONSTANTS[11]);
        public static final double WHEEL_BASE = Units.inchesToMeters(CHOSEN_CONSTANTS[12]);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                // Positive x values represent moving towards the front of the robot, positive y
                // values represent moving to the left
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front right wheel
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front left wheel
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Back left wheel
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Back right wheel
    }

    public static class VisionLimelight {
        // Offsets are from ROBOT to CAMERA, so if offset of 0.2, 0.2, 0.2, that means
        // camera is to the upwards top left of robot
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0);
        public static final double ROBOT_TO_CAMERA_OFFSET_FORWARD = Units.inchesToMeters(5.3);
        public static final double ROBOT_TO_CAMERA_OFFSET_SIDEWAYS = 0;
        public static final Transform2d CAMERA_TO_ROBOT_OFFSET = new Transform2d(ROBOT_TO_CAMERA_OFFSET_FORWARD, 0.0, new Rotation2d(0));
        public static final double CAMERA_MOUNT_DEGREES = 40;// change

        public static final double TARGET_HEIGHT_METERS = .5842; // change

        //public static final String CAMERA_NAME = "USB_2.0_1080P_Camera";
        //public static final double INTAKE_OFFSET = 0.3;
        public static final Translation3d cameraToRobotOffset = new Translation3d(
                Constants.VisionLimelight.ROBOT_TO_CAMERA_OFFSET_FORWARD,
                Constants.VisionLimelight.ROBOT_TO_CAMERA_OFFSET_SIDEWAYS,
                Constants.VisionLimelight.CAMERA_HEIGHT_METERS);
    }

    public static class Intake {
        public static final int INTAKE_MOTOR_1_PORT = 8;
        public static final int INTAKE_MOTOR_2_PORT = 9;

        public static final int INDEX_MOTOR_1_PORT = 53;
        public static final int INDEX_MOTOR_2_PORT = 54;
    }
}
