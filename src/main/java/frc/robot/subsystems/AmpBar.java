// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpBar extends SubsystemBase {

  private CANSparkMax ampBarMotor;
  private RelativeEncoder encoder;
  private SparkPIDController controller;

  public AmpBar() {
        ampBarMotor = new CANSparkMax(Constants.AmpBar.AMP_BAR_ID, MotorType.kBrushless);
        ampBarMotor.restoreFactoryDefaults();
        ampBarMotor.clearFaults();
        ampBarMotor.setSmartCurrentLimit(0, 15, 10);

        encoder = ampBarMotor.getEncoder();
        encoder.setPosition(0);
        controller = ampBarMotor.getPIDController();
  }

  public Command closedLoopControl(double setpoint){
    return new FunctionalCommand(
    ()-> {
        controller.setP(0.21);
        },
    ()-> controller.setReference(setpoint, CANSparkMax.ControlType.kPosition),
    interrupted-> {},
    ()-> Math.abs(encoder.getPosition() - setpoint) < 0.1,
    this
);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("amp bar encoder", encoder.getPosition());
  }
}
