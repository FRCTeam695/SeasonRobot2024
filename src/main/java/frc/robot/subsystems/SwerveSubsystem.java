package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule[] modules;

    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule bottomLeft;
    private final SwerveModule bottomRight;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;

    private final Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator odometry;
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {

        // Holds all the modules
        modules = new SwerveModule[4];

        // Creating the swerve modules
        frontRight = new SwerveModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0);
        frontLeft = new SwerveModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1);
        bottomLeft = new SwerveModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2);
        bottomRight = new SwerveModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3);

        modules[0] = frontRight;
        modules[1] = frontLeft;
        modules[2] = bottomLeft;
        modules[3] = bottomRight;

        camera = new PhotonCamera("Arducam_OV2311_USB_Camera");
        photonPoseEstimator = new PhotonPoseEstimator(
                                            Constants.Feild.APRIL_TAG_FIELD_LAYOUT, 
                                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                                            camera, 
                                            Constants.Vision.robotToCamera
                                        );

        odometry = new SwerveDrivePoseEstimator
                        (
                            Constants.Swerve.kDriveKinematics, 
                            getGyroHeading(),
                            getModulePositions(), 
                            new Pose2d()
                        );

        
        initAutoBuilder();
        initSwerve();

        SmartDashboard.putData("field", m_field);
    }


    /*
     * Sets the gyro at the beginning of the match and 
     * also sets each module state to present
     */
    public void initSwerve(){
        setRelativeTurnEncoderValues();
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                setGyro(180);
                setRelativeTurnEncoderValues();
            } catch (Exception e) {
            }
        }).start();
    }


    /*
     * Initializes autobuilder,
     * this is used by pathplanner
     */
    public void initAutoBuilder(){
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
    }


    /*
     * This method sets the gyro to 0 degrees
     * This method also sets the relative encoders on the swerve modules using the absolute encoders
     */
    public Command resetSwerve() {
        return runOnce(
            ()-> 
                {
                    if(isFlipped()) setGyro(180); // if we are on red alliance
                    else setGyro(0);
                    setRelativeTurnEncoderValues();
                }
        );
    }


    /*
     * This sets the gyro to a given angle,
     * it does this by resetting the gyro and then giving it an offset
     */
    public void setGyro(double degrees){
        gyro.reset();
        gyro.setAngleAdjustment(-degrees);
    }


    /*
     * getGyroHeading returns the angle the gyro is facing expressed as a Rotation2d
     */
    public Rotation2d getGyroHeading() {
        double gyroAngle = Math.IEEEremainder(gyro.getAngle(), 360);
        return new Rotation2d(gyroAngle * -Math.PI/180);
    }


    /*
     * isFlipped returns true if we are red alliance and returns false if we are blue alliance
     */
    public boolean isFlipped(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
        return false;
    }


    /*
     * returns the pose the odometry is currently reading
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }


    /*
     * Sets all the swerve modules to the states we want them to be in
     */
    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);

        for(SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.index]);
        }
        
    }


    /*
     * returns the position of each swerve module (check SwerveModule.java for further details)
     */
    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(SwerveModule module : modules){
            modulePositions[module.index] = module.getPosition();
        }

        return modulePositions;
    }


    /*
     * zeroes the drive motor encoders (check SwerveModule.java for further details)
     */
    public void zeroDriveMotorEncoders(){
        for(SwerveModule module : modules){
            module.zeroDrivePosition();
        }
    }


    /*
     * Uses the absolute encoders (cancoders) to set the relative encoders within each swerve module
     */
    public void setRelativeTurnEncoderValues() {

        for(SwerveModule module : modules){
            module.setTurnEncoder(module.getAbsoluteEncoderRadians());
        }

    }


    /*
     * Manually resets the odometry to a given pose
     */
    public void resetOdometry(Pose2d pose) {
        SmartDashboard.putString("adjustment", pose.toString());
        setGyro(pose.getRotation().getDegrees());
        odometry.resetPosition(
                getGyroHeading(),
                getModulePositions(),
                pose);
    }


    /*
     * Stops all of the swerve modules
     */
    public void stopModules() {
        for(SwerveModule module : modules){
            module.stop();
        }
    }


    /*
     * Returns the state of each swerve module
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(SwerveModule module : modules){
            states[module.index] = module.getState();
        }

        return states;
    }


    /*
     * Returns the current chassis speeds of the robot, 
     * used with pathplanner
     */
    public ChassisSpeeds getLatestChassisSpeed(){
        return Constants.Swerve.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }


    /*
     * Drives swerve given chassis speeds
     */
    public void driveSwerve(ChassisSpeeds chassisSpeeds) {
        // discretizes the chassis speeds (acccounts for robot skew)
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Constants.Swerve.DISCRETIZE_TIMESTAMP);

        // convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // set the modules to their desired speeds
        setModules(moduleStates);
    }


    public Command driveToPose(Pose2d finalPose){

        PathConstraints constraints = new PathConstraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, 
                                    2, 
                                                        Constants.Swerve.MAX_VELOCITY_RADIANS_PER_SECOND, 
                                                        Constants.Swerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
        
        return AutoBuilder.pathfindToPose(finalPose, constraints, 0, 0);
    }


    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            Optional<EstimatedRobotPose> poseEstimatorResult = photonPoseEstimator.update();

            if(!poseEstimatorResult.isEmpty()){
                Pose2d estimatedPose2d = poseEstimatorResult.get().estimatedPose.toPose2d();

                if(result.getTargets().size() > 1 || (result.getTargets().size() == 1 && result.getBestTarget().getPoseAmbiguity() < 0.2)){
                    odometry.addVisionMeasurement(estimatedPose2d, poseEstimatorResult.get().timestampSeconds);
                    SmartDashboard.putString("Vision Pose", estimatedPose2d.toString());
                }
            }
        }

        odometry.update( 
            getGyroHeading(),
            getModulePositions());

        m_field.setRobotPose(getPose());

        SmartDashboard.putNumber("gyro", getGyroHeading().getDegrees());
        SmartDashboard.putString("Robot Pose", odometry.getEstimatedPosition().toString());
    }


}
