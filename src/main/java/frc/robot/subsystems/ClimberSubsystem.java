// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;
  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;
  private float stringLengthLimit1;
  private float stringLengthLimit2;

  public ClimberSubsystem() {
    stringLengthLimit1 = (float)6.6;
    stringLengthLimit2 = (float)6.6;

    climberMotor1 = new CANSparkMax(Constants.Climber.CLIMBER_MOTOR_ID_1, MotorType.kBrushless);
    climberMotor2 = new CANSparkMax(Constants.Climber.CLIMBER_MOTOR_ID_2, MotorType.kBrushless);

    encoder1 = climberMotor1.getEncoder();
    encoder2 = climberMotor2.getEncoder();

    // per one revolution of the motor calculates exactly how much string is used
    encoder1.setPositionConversionFactor( (1/48.0) * (0.75 * Math.PI) );
    encoder2.setPositionConversionFactor( (1/48.0) * (0.75 * Math.PI) );

    climberMotor1.setIdleMode(IdleMode.kBrake);
    climberMotor2.setIdleMode(IdleMode.kBrake);

    setClimberMotorsSoftLimit(stringLengthLimit1, stringLengthLimit2);
  }

  public void setClimberMotorsSoftLimit(float limit1, float limit2){
    encoder1.setPosition(0);
    encoder2.setPosition(0);

    enableSoftLimits(true);

    climberMotor2.setSoftLimit(SoftLimitDirection.kForward, limit2);
    climberMotor2.setSoftLimit(SoftLimitDirection.kReverse, 0);

    climberMotor1.setSoftLimit(SoftLimitDirection.kForward, 0);
    climberMotor1.setSoftLimit(SoftLimitDirection.kReverse, -limit1);
  }

  public void enableSoftLimits(boolean toEnable){
    climberMotor1.enableSoftLimit(SoftLimitDirection.kForward, toEnable);
    climberMotor2.enableSoftLimit(SoftLimitDirection.kForward, toEnable);
    climberMotor1.enableSoftLimit(SoftLimitDirection.kReverse, toEnable);
    climberMotor2.enableSoftLimit(SoftLimitDirection.kReverse, toEnable);
  }

  public Command resetClimberMotorSoftLimit(){
    return runOnce(
      ()-> setClimberMotorsSoftLimit(stringLengthLimit1, stringLengthLimit2)
    );
  }

  public void turnOffSoftLimits(){
    enableSoftLimits(false);
  }

  public Command softLimitOverrideSlow(DoubleSupplier supplier1, DoubleSupplier supplier2){
    return 
    new FunctionalCommand(
      /*  INIT  */
      ()-> turnOffSoftLimits(),
      
      /* EXECUTE */
      ()-> 
            {
              climberMotor1.set(0.25 * supplier1.getAsDouble());
              climberMotor2.set(-0.25 * supplier2.getAsDouble());
            }, 
      
      /* END */
      interrupted-> setClimberMotorsSoftLimit(stringLengthLimit1, stringLengthLimit2), 

      /* END CONDITION */
      ()-> false, 
      this);
    // .andThen(
    //   run
    //     (
    //       ()-> 
    //         {
    //           climberMotor1.set(0.1 * supplier1.getAsDouble());
    //           climberMotor2.set(-0.1 * supplier2.getAsDouble());
    //         }
    //     )
    // )
    // .andThen(
    //   runOnce(()-> setClimberMotorsSoftLimit(stringLengthLimit))
    //   )
    // ;
  }

  public Command runClimberMotors(DoubleSupplier supplier1, DoubleSupplier supplier2){
    return run(
        ()->
            {
                climberMotor1.set(supplier2.getAsDouble());
                climberMotor2.set(-supplier1.getAsDouble());
            }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Motor 1", encoder1.getPosition());
    SmartDashboard.putNumber("Climb Motor 2", encoder2.getPosition());
  }
}
