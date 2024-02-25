// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;

  public ClimberSubsystem() {
    climberMotor1 = new CANSparkMax(Constants.Climber.CLIMBER_MOTOR_ID_1, MotorType.kBrushless);
    climberMotor2 = new CANSparkMax(Constants.Climber.CLIMBER_MOTOR_ID_2, MotorType.kBrushless);

    climberMotor1.setIdleMode(IdleMode.kBrake);
    climberMotor2.setIdleMode(IdleMode.kBrake);
  }

  public Command setClimberMotor1(double speed){
    return runOnce(
        ()-> climberMotor1.set(speed)
    );
  }

  public Command setClimberMotor2(double speed){
    return runOnce(
        ()-> climberMotor2.set(-speed)
    );
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
  }
}
