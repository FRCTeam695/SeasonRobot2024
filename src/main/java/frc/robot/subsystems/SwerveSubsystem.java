package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
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
    private final PIDController thetaController = new PIDController(0.05, 0, 0);

    private double prev_vel = 0;
    private double prev_timestamp = 0;
    private double max_accel = 0;

    private boolean rotationOverride = false;

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

        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        photonPoseEstimator = new PhotonPoseEstimator(
                                            Constants.Feild.APRIL_TAG_FIELD_LAYOUT, 
                                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                                            camera, 
                                            Constants.Vision.robotToCamera
                                        );

        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.005);
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0., 0., 0.);

        odometry = new SwerveDrivePoseEstimator
                        (
                            Constants.Swerve.kDriveKinematics, 
                            getGyroHeading(),
                            getModulePositions(), 
                            new Pose2d()
                            //stateStdDevs,
                            //visionStdDevs
                        );

        
        initAutoBuilder();
        initSwerve();

        thetaController.enableContinuousInput(-180, 180);
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
                    new PIDConstants(6.0, 0.1, 0.1), // Translation PID constants (JPK was 6,0,0)
                    new PIDConstants(5, 0.0, 0.0), // Rotation PID constants (JPK was 2)
                    Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, // Max module speed, in m/s
                    0.808, // Drive base radius in meters. Distance from robot center to furthest module.
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

    public void startRotationOverride(){
        rotationOverride = true;
    }

    public void endRotationOverride(){
        rotationOverride = false;
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

    public int getSpeakerId(){
        if(isFlipped()) return 4;
        else return 7;
    }

    public Double getDistanceToSpeakerTag(){
        var result = camera.getLatestResult();
        if(!result.hasTargets()) return null;
        
        else{
            // Get a list of currently tracked targets.
            List<PhotonTrackedTarget> targets = result.getTargets();

            for(PhotonTrackedTarget target : targets){
                if(target.getFiducialId() == getSpeakerId()){
                    Double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                    SmartDashboard.putNumber("distance", distance);
                    return distance;
                }
            }
        }
        return null;
    }

    public double getPitchToSpeaker(){
        double pitchFromSubwoofer = 1.05;
        double distanceFromSubwoofer = 0;

        double pitchFromPodium = Math.toRadians(40);
        double distanceFromPodium = 0;

        double y_intercept = 0;

        double slope_pitch = (pitchFromPodium - pitchFromSubwoofer)/(distanceFromPodium - distanceFromSubwoofer);

        Double distance = getDistanceToSpeakerTag();
        if(getDistanceToSpeakerTag() == null) return Math.toRadians(40);
        double convertedDistance = distance.doubleValue();
        return convertedDistance * slope_pitch + y_intercept;
    }

    public double getRPMToSpeaker(){
        double RPM_FromSubwoofer = 2222;
        double distanceFromSubwoofer = 0;

        double RPM_FromPodium = 3000;
        double distanceFromPodium = 0;

        double y_intercept = 0;

        double slope_RPM = (RPM_FromPodium - RPM_FromSubwoofer) / (distanceFromPodium - distanceFromSubwoofer);

        Double distance = getDistanceToSpeakerTag();
        if(getDistanceToSpeakerTag() == null) return Math.toRadians(40);
        double convertedDistance = distance.doubleValue();
        return convertedDistance * slope_RPM + y_intercept;
    }

    public Rotation2d getRotationToSpeaker(){
        var result = camera.getLatestResult();
        if(!result.hasTargets()) return null;
        else{
            // Get a list of currently tracked targets.
            List<PhotonTrackedTarget> targets = result.getTargets();

            for(PhotonTrackedTarget target : targets){
                if(target.getFiducialId() == getSpeakerId()) {
                    double yaw = target.getYaw();
                    SmartDashboard.putNumber("Yaw", yaw);
                    return new Rotation2d(yaw * Math.PI/180);
                };
            }
        }
        return null;
    }


    /*
     * Drives swerve given chassis speeds
     */
    public void driveSwerve(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds newSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        if(rotationOverride){
            SmartDashboard.putBoolean("override", true);
            Rotation2d rotation = getRotationToSpeaker();
            if(rotation != null){ // If we can see the speaker tag
                double currentRotation = getGyroHeading().getDegrees();
                double ROTATION_PID_OUTPUT = thetaController.calculate(currentRotation, currentRotation - rotation.getDegrees());
                SmartDashboard.putNumber("CURRENT ROTATION", currentRotation);
                SmartDashboard.putNumber("SETPOINT ROTATION", currentRotation - rotation.getDegrees());
                SmartDashboard.putNumber("ROTATION_PID_OUTPUT", ROTATION_PID_OUTPUT);
                newSpeeds = new ChassisSpeeds(
                                chassisSpeeds.vxMetersPerSecond, 
                                chassisSpeeds.vyMetersPerSecond, 
                                MathUtil.clamp(ROTATION_PID_OUTPUT, -1, 1) * Constants.Swerve.MAX_ANGULAR_SPEED_METERS_PER_SECOND
                            );
            }
        }else{
            SmartDashboard.putBoolean("override", false);
        }
        
        // discretizes the chassis speeds (acccounts for robot skew)
        newSpeeds = ChassisSpeeds.discretize(newSpeeds, Constants.Swerve.DISCRETIZE_TIMESTAMP);

        // convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(newSpeeds);

        // set the modules to their desired speeds
        setModules(moduleStates);
    }

    public Pose2d flipPose(Pose2d pose){
        return new Pose2d(Math.abs(16.54 - pose.getX()), pose.getY(), pose.getRotation());
    }


    public Command driveToPose(Pose2d finalPose){
    
    // There is a negation because all poses were calculated from red side
    if(!isFlipped()) finalPose = flipPose(finalPose);

        PathConstraints constraints = new PathConstraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, 
                                    2, 
                                                        Constants.Swerve.MAX_VELOCITY_RADIANS_PER_SECOND, 
                                                        Constants.Swerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
        
        return AutoBuilder.pathfindToPose(finalPose, constraints, 0, 0);
    }

    public Matrix<N3, N1> getNewVisionStdDevs(double distanceToTags){
        SmartDashboard.putNumber("distance", distanceToTags);
        double slope_x = 0.025;
        double slope_y = 0.025;

        double slope_theta = 1000;

        return VecBuilder.fill(
            slope_x * distanceToTags,
            slope_y * distanceToTags,
            slope_theta * distanceToTags
        );
    }

    public void updateVision(){
        var result = camera.getLatestResult();

        if(result.hasTargets()) { //}  && getPose().getX() >= 12.5){
            // double cameraToTagDistance = result.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());

            Optional<EstimatedRobotPose> poseEstimatorResult = photonPoseEstimator.update(result);

            if(poseEstimatorResult.isPresent()){
                var estimatedPose = poseEstimatorResult.get();
                var estimatedPose2d = estimatedPose.estimatedPose.toPose2d();
                var targets = estimatedPose.targetsUsed;

                

                // Translation2d visionTranslation = estimatedPose2d.getTranslation();
                // Translation2d currentEstimatedTranslation = getPose().getTranslation();

                // double visionToCurrentPoseError = visionTranslation.getDistance(currentEstimatedTranslation);
                // SmartDashboard.putNumber("distance", cameraToTagDistance);
                // SmartDashboard.putNumber("error", visionToCurrentPoseError);
                if((result.getTargets().size() > 1) && targets != null){ //){
                    double minDistance = Double.MAX_VALUE;
                    double maxDistance = 0.;
                    double maxAmbiguity = 0.;

                    for(PhotonTrackedTarget target : targets){
                        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                        double ambiguity = target.getPoseAmbiguity();

                        if(distance < minDistance) minDistance = distance;
                        if(distance > maxDistance) maxDistance = distance;
                        if(ambiguity > maxAmbiguity) maxAmbiguity = ambiguity;

                        SmartDashboard.putNumber("Target #" + target.getFiducialId(), ambiguity);
                    }

                    SmartDashboard.putNumber("Distance min", minDistance);
                    SmartDashboard.putNumber("Distance max", maxDistance);

                    SmartDashboard.putNumber("Max Ambiguity", maxAmbiguity);
                    if(maxAmbiguity < 0.2){
                        // odometry.addVisionMeasurement(
                        //         estimatedPose2d, 
                        //         poseEstimatorResult.get().timestampSeconds//,
                                //getNewVisionStdDevs(minDistance)
                        //getNewVisionStdDevs(cameraToTagDistance)
                    //);
                        // override vision rotation with gyro
                        // odometry.addVisionMeasurement(
                        //     new Pose2d(estimatedPose2d.getX(), estimatedPose2d.getY(), getGyroHeading()), 
                        //     poseEstimatorResult.get().timestampSeconds);
                    }
                    
                    // // original
                    // odometry.addVisionMeasurement(
                    //    estimatedPose2d, 
                    //    poseEstimatorResult.get().timestampSeconds,
                    //    getNewVisionStdDevs(minDistance)
                    //     //getNewVisionStdDevs(cameraToTagDistance)
                    // );

                    SmartDashboard.putString("Vision Pose", estimatedPose2d.toString());
                }
            }
        }
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Red", isFlipped());
        ChassisSpeeds speeds = getLatestChassisSpeed();
        double velocity = Math.sqrt((speeds.vxMetersPerSecond * speeds.vxMetersPerSecond) + (speeds.vyMetersPerSecond * speeds.vyMetersPerSecond));
        double accel = (velocity - prev_vel)/ (Timer.getFPGATimestamp() - prev_timestamp);
        if(accel > max_accel){
            max_accel = accel;
        }

        prev_vel = velocity;
        prev_timestamp = Timer.getFPGATimestamp();

        odometry.update( 
            getGyroHeading(),
            getModulePositions());

        //updateVision();

        m_field.setRobotPose(getPose());

        SmartDashboard.putNumber("gyro", getGyroHeading().getDegrees());
        SmartDashboard.putNumber("max acceleration", max_accel);
        SmartDashboard.putNumber("Current acceleration", accel);
        SmartDashboard.putString("Robot Pose", odometry.getEstimatedPosition().toString());

        SmartDashboard.putBoolean("Rotation Override", rotationOverride);
    }


}
