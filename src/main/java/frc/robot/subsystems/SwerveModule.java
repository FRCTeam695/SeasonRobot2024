package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import frc.robot.Constants;

public class SwerveModule{
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private double setPoint;

    private final WPI_CANCoder absoluteEncoder;

    private final PIDController turningPidController;

    private final double absoluteEncoderOffset;
    private final double turningGearRatio;
    private final double drivingGearRatio;
    private final double maxSpeedMPS;
    private final double wheelCircumference;


    public SwerveModule(int driveMotorId, int turnMotorId, double absoluteEncoderOffset, int TurnCANCoderId){
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        //Current limit to the falcons
        SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
        falconlimit.enable = true;
        falconlimit.currentLimit = 20;
        falconlimit.triggerThresholdCurrent = 20;
        falconlimit.triggerThresholdTime = 0;

        //Creates and configs drive motor
        driveMotor = new TalonFX(driveMotorId, "drivetrain");
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configSupplyCurrentLimit(falconlimit);

        //Creates and configs turn motor
        turnMotor = new TalonFX(turnMotorId, "drivetrain");
        turnMotor.configFactoryDefault();
        turnMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.configSupplyCurrentLimit(falconlimit);

        //Absolute encoder
        absoluteEncoder = new WPI_CANCoder(TurnCANCoderId, "drivetrain");

        //Creates the PID controller for turning
        //turningPidController = new PIDController(0.015, 0.0, 0.0);
        turningPidController = new PIDController(Constants.Swerve.THETA_KP_VALUE, 0.0, 0.0);
        turningPidController.enableContinuousInput(-180, 180); //Tells the PID controller that 180 and -180 are at the same place

        //Setpoint is used cus when wheel turn ccw abs encoder turn ccw but relative encoder turn cc
        //(No idea what I was thinking when I wrote this comment but just roll w/ it if u got questions just ask me)
        setPoint = 7.0; //This is impossible bcs its in radians so it can't reach that high yk

        //Constants
        turningGearRatio = Constants.Swerve.TURNING_GEAR_RATIO;
        drivingGearRatio = Constants.Swerve.DRIVING_GEAR_RATIO;
        maxSpeedMPS = Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
        wheelCircumference = Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;

    }


    //getTurnPosition calculates the angle from the angle returned by the relative encoder
    public double getTurnPosition(boolean degrees) {

        if (setPoint == 7){  //Set to 7 in constructor, means relative encoder hasn't been set yet, this is a safety
            return -1;
        }

        double turnDegreeValue = turnMotor.getSelectedSensorPosition() % (2048 * turningGearRatio) / (2048 * turningGearRatio) * 360;

        if(Constants.Swerve.isMK4i == 1){
            turnDegreeValue = (setPoint * 180 / Math.PI) + ((setPoint * 180 / Math.PI) - turnDegreeValue); //Adjusts the value cause the falcon encoder is upside down compared to the wheel
        }

        //Binds the value between -180 and 180 degrees
        while (turnDegreeValue > 180) {
            turnDegreeValue -= 360;
        }

        while (turnDegreeValue < -180) {
            turnDegreeValue += 360;
        }

        if (degrees){
            return turnDegreeValue;
        }

        return turnDegreeValue * Math.PI / 180;
    }

    //falconToRps converts falcon ticks to rotations per second of the wheel
    public double falconToRPS(double velocityCounts, double gearRatio) {
        double motorRPS = velocityCounts * (10.0 / 2048.0);        
        double mechRPS = motorRPS / gearRatio * -1;  //multiply by negative 1 bcs the falcon is upside down
        return mechRPS;
    }

    //degreesToFalcon converts degrees to falcon ticks
    public double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    //Gets the drive velocity of the robot in meters per second
    public double getDriveVelocity() {
        double wheelRPS = falconToRPS(driveMotor.getSelectedSensorVelocity(), drivingGearRatio);
        double wheelMPS = (wheelRPS * wheelCircumference);
        return wheelMPS;
    }

    //Gets the absolute encoder value in radians
    public double getAbsoluteEncoderRadians() {
        double raw_val = absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset;

        //Binds the value between -180 and 180 degrees
        while (raw_val > 180) {
            raw_val -= 360;
        }
        while (raw_val < -180) {
            raw_val += 360;
        }

        return raw_val * Math.PI / 180; //Converts to radians
    }

    //Gets the velocity of the wheel and the direction the wheel is turned to
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition(false)));
    }

    //Converts falcon ticks to meters
    public double falconToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    //getPosition returns the position of the wheel, similar too a SwerveModuleState
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            falconToMeters(driveMotor.getSelectedSensorPosition(), wheelCircumference, drivingGearRatio), 
            getState().angle
        );
    }

    //zeroDrivePosition sets the drive motor encoders to 0
    public void zeroDrivePosition(){
        driveMotor.setSelectedSensorPosition(0);
    }

    //setDesiredState sets the swerve modules to a given state
    public void setDesiredState(SwerveModuleState state, int motor) {
        if (Math.abs(state.speedMetersPerSecond) < 0.1){
           stop();
           return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        
        double setpoint = state.angle.getDegrees();
        double turnMotorOutput = 0;

        if(Constants.Swerve.isMK4i == 1){
            turnMotorOutput = -1 * MathUtil.clamp(turningPidController.calculate(getState().angle.getDegrees(), setpoint), -1, 1);
        }
        else{
            turnMotorOutput =  MathUtil.clamp(turningPidController.calculate(getState().angle.getDegrees(), setpoint), -1, 1);
        }
        //Multiply by -1 above because the falcon is upside down compared to the wheel on MK4i's
        
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / maxSpeedMPS);
        turnMotor.set(ControlMode.PercentOutput, turnMotorOutput);
    }

    //stops the drive and turn motors
    public void stop(){
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    //setTurnEncoder sets the relative turn encoder to a certain radian value (Usually an angle given by the absolute encoder)
    public void setTurnEncoder(double radians) {
        double desired_ticks = radians / Math.PI * (1024 * turningGearRatio);
        turningPidController.reset();
        setPoint = radians;  //Before this line is executed setPoint is equal to 7
        turnMotor.setSelectedSensorPosition(desired_ticks);
    }
    
    //gets the position of the drive motor
    public double getDriveTicks(){
        return driveMotor.getSelectedSensorPosition();
    }

}