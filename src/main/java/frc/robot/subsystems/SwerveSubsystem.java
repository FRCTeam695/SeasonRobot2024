package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule bottomLeft;
    private final SwerveModule bottomRight;

    private final Timer timer = new Timer();
    private final Field2d m_field = new Field2d();

    // creates the odometry class
    public SwerveDrivePoseEstimator odometry;

    // initialize the gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private double gyroAngle;


    public double tickStart;
    public ChassisSpeeds latestChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    public SwerveSubsystem() {

        // Creating the swerve modules
        frontRight = new SwerveModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET, Constants.Swerve.FRONT_RIGHT_CANCODER_ID);
        frontLeft = new SwerveModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET, Constants.Swerve.FRONT_LEFT_CANCODER_ID);
        bottomLeft = new SwerveModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET, Constants.Swerve.BACK_LEFT_CANCODER_ID);
        bottomRight = new SwerveModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET, Constants.Swerve.BACK_RIGHT_CANCODER_ID);

        SwerveDriveKinematics driveKinematics = Constants.Swerve.kDriveKinematics;
        odometry = new SwerveDrivePoseEstimator(driveKinematics, new Rotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(),
                        bottomLeft.getPosition(), bottomRight.getPosition() }, new Pose2d());

        tickStart = 0;


        SmartDashboard.putData("field", m_field);

        // jpk add:
        setRelativeTurnEncoderValue();  // sync the relative encoders (falcons) with the absolute encoders (cancoders)

        // Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getLatestChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveSwerve, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(3.0, 0.0, 0), // Translation PID constants
                new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, // Max module speed, in m/s
                0.3, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(false, true) // Default path replanning config. See the API for the options here
            ),
            this::isFlipped,
        this // Reference to this subsystem to set requirements
    );

        // resets the gyro, it is calibrating when this code is reached so we reset it
        // on a different thread with a delay
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    //Sets the gyro heading to 0
    public void zeroHeading() {
        gyro.reset();
    }

    //getPitch is used for balancing the robot on the charge station
    public double getPitch() {
        return gyro.getPitch();
    }

    //getHeading returns the direction the robot is facing
    //assumes we have already reset the gyro
    public double getHeading() {
        gyroAngle = Math.IEEEremainder(gyro.getAngle(), 360);

        return gyroAngle * -1; // Multiply by negative one because on wpilib as you go counterclockwise angles
                          // should get bigger
    }

    //tells pathplanner if it should flip the paths or not, this is dependant on what aliance we are on and also what game we are playing
    private boolean isFlipped(){
        return true;
    }

    //Outdated method for using pure odometry to make autons
    public void startTickCount() {
        tickStart = frontLeft.getDriveTicks();
    }

    //Outdated method for using pure odometry to make autons
    public double getTicks() {
        return Math.abs(frontLeft.getDriveTicks() - tickStart);
    }

    //Gets the pose of the robot from the odometry, used in any self driving periods
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    //Sets all of the swerve modules
    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);

        /*
        SmartDashboard.putString("SWERVE M STATE FR", desiredStates[0].toString());
        SmartDashboard.putString("SWERVE M STATE FL", desiredStates[1].toString());
        SmartDashboard.putString("SWERVE M STATE BL", desiredStates[2].toString());
        SmartDashboard.putString("SWERVE M STATE BR", desiredStates[3].toString());
        */

        frontRight.setDesiredState(desiredStates[0], 1);
        frontLeft.setDesiredState(desiredStates[1], 2);
        bottomLeft.setDesiredState(desiredStates[2], 3);
        bottomRight.setDesiredState(desiredStates[3], 4);
        
    }

    //returns the position of each swerve module (check SwerveModule.java for further details)
    public SwerveModulePosition getModulePosition(int motor) {
        switch (motor) {
            case 1:
                return frontRight.getPosition();
            case 2:
                return frontLeft.getPosition();
            case 3:
                return bottomLeft.getPosition();
            case 4:
                return bottomRight.getPosition();
            default:
                return bottomLeft.getPosition();
        }
    }

    //Gets the position of the wheel as returned by the absolute encoder (check SwerveModule.java for further details)
    public double getAbsoluteEncoderValue(int motor) {
        /*
        SmartDashboard.putNumber("FRONT LEFT MOTOR POSITION ABS", frontLeft.getAbsoluteEncoderRadians() * 180/Math.PI);
        SmartDashboard.putNumber("FRONT RIGHT MOTOR POSITION ABS", frontRight.getAbsoluteEncoderRadians() * 180/Math.PI);
        SmartDashboard.putNumber("BACK LEFT MOTOR POSITION ABS", bottomLeft.getAbsoluteEncoderRadians() * 180/Math.PI);
        SmartDashboard.putNumber("BACK RIGHT MOTOR POSITION ABS", bottomRight.getAbsoluteEncoderRadians() * 180/Math.PI);
        */

        switch (motor) {
            case 1:
                return frontRight.getAbsoluteEncoderRadians();
            case 2:
                return frontLeft.getAbsoluteEncoderRadians();
            case 3:
                return bottomLeft.getAbsoluteEncoderRadians();
            case 4:
                return bottomRight.getAbsoluteEncoderRadians();
            default:
                return -1;
        }
    }

    //Gets the swerve module state of the motor (check SwerveModule.java for further details)
    public SwerveModuleState getState(int motor) {
        switch (motor) {
            case 1:
                return frontRight.getState();
            case 2:
                return frontLeft.getState();
            case 3:
                return bottomLeft.getState();
            case 4:
                return bottomRight.getState();
            default:
                return bottomRight.getState(); // just bcs I need a default
        }
    }

    //Gets the relative encoder position of each swerve module (check SwerveModule.java for further details)
    public double getRelativeTurnEncoderValue(int motor) {

        /*
        SmartDashboard.putNumber("FRONT LEFT MOTOR POSITION", frontLeft.getTurnPosition(true));
        SmartDashboard.putNumber("FRONT RIGHT MOTOR POSITION", frontRight.getTurnPosition(true));
        SmartDashboard.putNumber("BACK LEFT MOTOR POSITION", bottomLeft.getTurnPosition(true));
        SmartDashboard.putNumber("BACK RIGHT MOTOR POSITION", bottomRight.getTurnPosition(true));
        */

        switch (motor) {
            case 1:
                return frontRight.getTurnPosition(false);
            case 2:
                return frontLeft.getTurnPosition(false);
            case 3:
                return bottomLeft.getTurnPosition(false);
            case 4:
                return bottomRight.getTurnPosition(false);
            default:
                return -1;
        }
    }

    // resets the drive motors (check SwerveModule.java for further details)
    public void resetDriveMotorEncoders(){
        frontRight.zeroDrivePosition();
        frontLeft.zeroDrivePosition();
        bottomLeft.zeroDrivePosition();
        bottomRight.zeroDrivePosition();
    }

    //Sets the relative encoder value using the absolute encoders (check SwerveModule.java for further details)
    public void setRelativeTurnEncoderValue() {
        frontRight.setTurnEncoder(frontRight.getAbsoluteEncoderRadians());
        frontLeft.setTurnEncoder(frontLeft.getAbsoluteEncoderRadians());
        bottomLeft.setTurnEncoder(bottomLeft.getAbsoluteEncoderRadians());
        bottomRight.setTurnEncoder(bottomRight.getAbsoluteEncoderRadians());
    }

    //periodic updates the odometry object
    @Override
    public void periodic() {


        if(LimelightHelpers.getLatestResults("").targetingResults.targets_Fiducials.length >= 2){
            odometry.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(""), timer.getFPGATimestamp());
        }

        odometry.updateWithTime(timer.getFPGATimestamp(), new Rotation2d(getHeading() * Math.PI / 180),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(),
                        bottomLeft.getPosition(), bottomRight.getPosition() });
        m_field.setRobotPose(getPose());

        SmartDashboard.putString("Robot Pose", odometry.getEstimatedPosition().toString());
    }

    //Allows us to manually reset the odometer, used with vision pose estimation
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontRight.getPosition(),
                        frontLeft.getPosition(),
                        bottomLeft.getPosition(),
                        bottomRight.getPosition()
                },
                pose);

    }

    // Stops all of the swerve modules
    public void stopModules() {
        frontRight.stop();
        frontLeft.stop();
        bottomLeft.stop();
        bottomRight.stop();
    }

    // Drives from current location to specified pose
    public Command goToLocation(Pose2d finalpose){

        Pose2d currentPose = getPose();
        
        // Create a list of bezier points from poses. Each pose represents one waypoint. 
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromRadians(Math.atan((Math.abs(currentPose.getY()-finalpose.getY())/(Math.abs(currentPose.getX()-currentPose.getY())))))),
            new Pose2d(finalpose.getX(), finalpose.getY(), Rotation2d.fromDegrees(finalpose.getRotation().getDegrees()))
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, 3.0, Constants.Swerve.MAX_VELOCITY_RADIANS_PER_SECOND, Constants.Swerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return AutoBuilder.followPath(path);
    }

    public ChassisSpeeds getLatestChassisSpeed(){
        return latestChassisSpeeds;
    }

    // method that actually drives swerve
    public void driveSwerve(ChassisSpeeds chassisSpeeds) {
        // discretizes the chassis speeds (acccounts for robot skew) The timestamp should be the time 
        // between each execute in the command is called.
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Constants.Swerve.DISCRETIZE_TIMESTAMP);

        latestChassisSpeeds = chassisSpeeds;

        // convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // set the modules to their desired speeds
        setModules(moduleStates);
    }

}
