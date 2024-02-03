// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsubsystem extends SubsystemBase {

  private final CANSparkFlex shooterNeo1 = new CANSparkFlex(51, MotorType.kBrushless);
  private final CANSparkFlex shooterNeo2 = new CANSparkFlex(52, MotorType.kBrushless);

  // private final CANSparkFlex armNeo = new CANSparkFlex(0,
  // MotorType.kBrushless);

  private final RelativeEncoder shooterNeoEncoder1 = shooterNeo1.getEncoder();
  private final RelativeEncoder shooterNeoEncoder2 = shooterNeo2.getEncoder();

  private final SparkPIDController shooterNeo1PID = shooterNeo1.getPIDController();
  private final SparkPIDController shooterNeo2PID = shooterNeo2.getPIDController();

  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsubsystem() {
    shooterNeoEncoder1.setPosition(0);
    shooterNeoEncoder2.setPosition(0);

    // PID coefficients
    kP = 0.000100;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000155;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

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

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  /**
   * setSpeedCommand
   *
   * @return a command
   */
  public void setSpeed(DoubleSupplier speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    // double voltage2 = voltage.getAsDouble();
    shooterNeo1.set(-speed.getAsDouble());
    shooterNeo2.set(speed.getAsDouble());

  }

  public boolean shooterIsUpToSpeed(){
    if(Math.abs(setPoint-shooterNeoEncoder2.getVelocity()) <= 10 && Math.abs(setPoint+shooterNeoEncoder1.getVelocity()) <= 10)
    {
      return true;
    }
    return false;
  }

  public Command runVelocity(DoubleSupplier velocity) {
    return run(
        () -> {
          // double p = SmartDashboard.getNumber("P Gain", 0);
          // double i = SmartDashboard.getNumber("I Gain", 0);
          // double d = SmartDashboard.getNumber("D Gain", 0);
          // double iz = SmartDashboard.getNumber("I Zone", 0);
          // double ff = SmartDashboard.getNumber("Feed Forward", 0);
          // double max = SmartDashboard.getNumber("Max Output", 0);
          // double min = SmartDashboard.getNumber("Min Output", 0);

          // // if PID coefficients on SmartDashboard have changed, write new values to
          // controller
          // if((p != kP)) { neo1PID.setP(p); neo2PID.setP(p); kP = p; }
          // if((i != kI)) { neo1PID.setI(i); neo2PID.setI(i); kI = i; }
          // if((d != kD)) { neo1PID.setD(d); neo2PID.setD(d); kD = d; }
          // if((iz != kIz)) { neo1PID.setIZone(iz); neo2PID.setIZone(iz); kIz = iz; }
          // if((ff != kFF)) { neo1PID.setFF(ff); neo2PID.setFF(ff); kFF = ff; }
          // if((max != kMaxOutput) || (min != kMinOutput)) {
          // neo1PID.setOutputRange(min, max);
          // neo2PID.setOutputRange(min, max);
          // kMinOutput = min; kMaxOutput = max;
          // }

          setPoint = velocity.getAsDouble() * maxRPM;
          shooterNeo1PID.setReference(-setPoint, CANSparkMax.ControlType.kVelocity);
          shooterNeo2PID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

          SmartDashboard.putNumber("SetPoint", setPoint);
        });
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
    SmartDashboard.putNumber("Encoder 1 Velocity", -shooterNeoEncoder1.getVelocity());
    SmartDashboard.putNumber("Encoder 2 Velocity", shooterNeoEncoder2.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}