// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkFlex shooterNeo1 = new CANSparkFlex(Constants.Shooter.SHOOTER_MOTOR_ID_1, MotorType.kBrushless);
  private final CANSparkFlex shooterNeo2 = new CANSparkFlex(Constants.Shooter.SHOOTER_MOTOR_ID_2, MotorType.kBrushless);

  // private final CANSparkFlex armNeo = new CANSparkFlex(0,
  // MotorType.kBrushless);

  private final RelativeEncoder shooterNeoEncoder1 = shooterNeo1.getEncoder();
  private final RelativeEncoder shooterNeoEncoder2 = shooterNeo2.getEncoder();

  private final SparkPIDController shooterNeo1PID = shooterNeo1.getPIDController();
  private final SparkPIDController shooterNeo2PID = shooterNeo2.getPIDController();

  private String scoringStatus;

  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPointRPM;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {

    shooterNeo1.restoreFactoryDefaults();
    shooterNeo2.restoreFactoryDefaults();

    shooterNeo1.clearFaults();
    shooterNeo2.clearFaults();

    shooterNeoEncoder1.setPosition(0);
    shooterNeoEncoder2.setPosition(0);

    shooterNeo1.setIdleMode(IdleMode.kBrake);
    shooterNeo2.setIdleMode(IdleMode.kBrake);

    shooterNeo1.enableVoltageCompensation(11.5);
    shooterNeo2.enableVoltageCompensation(11.5);
    // PID coefficients
    kP = 0.000150;// * 2; **times two for amp
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000155;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;
    setPointRPM = 0;

    // set PID coefficients
    shooterNeo1PID.setP(kP);
    shooterNeo1PID.setI(kI);
    shooterNeo1PID.setD(kD);
    shooterNeo1PID.setIZone(kIz);
    shooterNeo1PID.setFF(kFF);
    shooterNeo1PID.setOutputRange(kMinOutput, kMaxOutput);

    shooterNeo2PID.setP(kP);
    shooterNeo2PID.setI(kI);
    shooterNeo2PID.setD(kD);
    shooterNeo2PID.setIZone(kIz);
    shooterNeo2PID.setFF(kFF);
    shooterNeo2PID.setOutputRange(kMinOutput, kMaxOutput);

    scoringStatus = "speaker";

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0);
  }

  // Used in order to know what speed to shoot at, i.e. amp vs speaker
  public Command setScoringStatus(String newStatus){
    return runOnce(
    ()-> {scoringStatus = newStatus;}
    );
  }

  // Returns the scoring status
  public String getScoringStatus(){
    return scoringStatus;
  }

  /**
   * setSpeedCommand
   *
   * @return a command
   */
  public void setPercentVBus(DoubleSupplier percentVBus) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    // double voltage2 = voltage.getAsDouble();
    shooterNeo1.set(-percentVBus.getAsDouble());
    shooterNeo2.set(percentVBus.getAsDouble());

  }

  public boolean currentAboveTwentyAmps(){
    return (shooterNeo1.getOutputCurrent() > 20);// || (shooterNeo2.getOutputCurrent() > 20);
  }

  public boolean currentBelowTwentyAmps(){
    return !currentAboveTwentyAmps();
  }

  // returns true id the shooter is up to speed
  public boolean shooterIsUpToSpeed(){

    double deadband = setPointRPM * (75/2222.0); //unit is RPM
    if(Math.abs(setPointRPM-Math.abs(shooterNeoEncoder2.getVelocity())) <= deadband 
    && Math.abs(setPointRPM-Math.abs(shooterNeoEncoder1.getVelocity())) <= deadband)
    {
      return true;
    }
    return false;
  }

  // returns true if the shooter is not up to speed
  public boolean shooterIsNotUpToSpeed(){
    return !shooterIsUpToSpeed();
  }

  // checks if the shooter is running or not
  public boolean isRunning(){
    return setPointRPM > 5;
  }

  public Command shooterDefaultCommand(double kp_rot){
    return new FunctionalCommand(
      ()-> {
        shooterNeo1PID.setIAccum(0);
        shooterNeo2PID.setIAccum(0);

        // makes sure to set the kp, kp for shooting vs closed loop rotation is very different
        shooterNeo1PID.setP(1);
        shooterNeo2PID.setP(1);

        shooterNeo1PID.setI(0);
        shooterNeo2PID.setI(0);
        
        // this is just for good measure
        shooterNeo1PID.setOutputRange(-1, 1);
        shooterNeo2PID.setOutputRange(-1, 1);

        //resets the encoders
        shooterNeoEncoder1.setPosition(0);
        shooterNeoEncoder2.setPosition(0);
      },

      ()-> {
        // tells the neo's where to go
        shooterNeo1PID.setReference(0, CANSparkMax.ControlType.kPosition);
        shooterNeo2PID.setReference(0, CANSparkMax.ControlType.kPosition);
      },

      interrupted-> {},

      // end condition
      ()-> (false)

      // ALWAYS REQUIRE THE SUBSYSTEM!!
    , this);
  }

  // rotates a specific amount of rotations using closed loop control
  public Command closedLoopRotation(double rotations, double kp_rot, double ki_rot){

    return new FunctionalCommand(
      ()-> {
        shooterNeo1PID.setIAccum(0);
        shooterNeo2PID.setIAccum(0);

        // makes sure to set the kp, kp for shooting vs closed loop rotation is very different
        shooterNeo1PID.setP(kp_rot);
        shooterNeo2PID.setP(kp_rot);

        shooterNeo1PID.setI(ki_rot);
        shooterNeo2PID.setI(ki_rot);
        
        // this is just for good measure
        shooterNeo1PID.setOutputRange(-1, 1);
        shooterNeo2PID.setOutputRange(-1, 1);

        //resets the encoders
        shooterNeoEncoder1.setPosition(0);
        shooterNeoEncoder2.setPosition(0);
      },

      ()-> {
        // tells the neo's where to go
        shooterNeo1PID.setReference(rotations, CANSparkMax.ControlType.kPosition);
        shooterNeo2PID.setReference(-rotations, CANSparkMax.ControlType.kPosition);
      },

      interrupted-> {
        shooterNeo1PID.setI(0);
        shooterNeo2PID.setI(0);
      },

      // end condition
      ()-> (Math.abs(shooterNeoEncoder1.getPosition()) >= rotations * 0.95)

      // ALWAYS REQUIRE THE SUBSYSTEM!!
    , this);
  }


  public Command runVelocity(DoubleSupplier velocity) {
    return new FunctionalCommand(
        ()-> {
          shooterNeo1PID.setIAccum(0);
          shooterNeo2PID.setIAccum(0);

          shooterNeo1PID.setP(kP);
          shooterNeo2PID.setP(kP);

          shooterNeo1PID.setI(0);
          shooterNeo2PID.setI(0);
        },
        () -> {
          setPointRPM = velocity.getAsDouble() * maxRPM;
          shooterNeo1PID.setReference(setPointRPM, CANSparkMax.ControlType.kVelocity);
          shooterNeo2PID.setReference(-setPointRPM, CANSparkMax.ControlType.kVelocity);

          SmartDashboard.putNumber("SetPoint", setPointRPM);
        },
        interrupted-> {},
        ()-> (false),
        this
        
        );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Position 2", shooterNeoEncoder2.getPosition());
    //SmartDashboard.putNumber("Position 1", shooterNeoEncoder1.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}