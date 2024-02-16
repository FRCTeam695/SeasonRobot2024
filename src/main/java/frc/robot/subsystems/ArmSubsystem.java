// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private ProfiledPIDController m_controller;
  private ArmFeedforward m_feedForward;

  private DigitalInput armDI;
  private DutyCycleEncoder m_encoder;

  private double prevGoalVelocity;

  public ArmSubsystem() {
    armDI = new DigitalInput(Constants.Arm.ARM_ENCODER_PORT);
    m_encoder = new DutyCycleEncoder(armDI);

    m_controller = 
        new ProfiledPIDController(
            Constants.Arm.KP,
            Constants.Arm.KI, 
            Constants.Arm.KD, 
            new TrapezoidProfile.Constraints(Constants.Arm.MAX_VELOCITY, Constants.Arm.MAX_ACCELERATION));
    
    m_feedForward = 
        new ArmFeedforward(
            Constants.Arm.KS, 
            Constants.Arm.KG, 
            Constants.Arm.KV, 
            Constants.Arm.KA);

    prevGoalVelocity = 0;
  }

  private void reachGoal(double goal){
    m_controller.setGoal(goal);

    double velocity = m_controller.getSetpoint().velocity;
    double acceleration = (velocity - prevGoalVelocity) / m_controller.getPeriod();

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getAbsolutePosition());
    double feedforwardOutput = m_feedForward.calculate(m_encoder.getAbsolutePosition(), velocity, acceleration);

    SmartDashboard.putNumber("PID OUTPUT", pidOutput);
    SmartDashboard.putNumber("FEEDFORWARD OUTPUT", feedforwardOutput);

    
  }
}
