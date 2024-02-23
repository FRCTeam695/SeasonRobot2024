// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_pitchControlMotor;

  private ProfiledPIDController m_controller;
  private ArmFeedforward m_feedForward;

  private DigitalInput armDI;
  private DutyCycleEncoder m_encoder;

  private double prevGoalVelocity;
  private double goal;

  public ArmSubsystem() {
    m_pitchControlMotor = new CANSparkMax(Constants.Arm.PITCH_MOTOR_ID, MotorType.kBrushless);
    m_pitchControlMotor.setSmartCurrentLimit(20);

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
    goal = 0;
  }

  /*
  public Command goToAngle(double goal){
    
    return run(()-> reachGoal(goal));
  }
  */

  public void setGoal(double newGoal){
    goal = newGoal;
  }

  private double getAbsolutePosition(){
    return 2 * Math.PI * (Constants.Arm.ABSOLUTE_ENCODER_OFFSET - m_encoder.getAbsolutePosition()) / 3;
  }

  public boolean atGoal(){
    return Math.abs(getAbsolutePosition() - goal) <= 0.05;
  }

  public void resetStateToPresent()
  {
    m_controller.reset(getAbsolutePosition());
  }
    

  private void reachGoal(double goal){
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    // Outputs are negated because of encoder position
    double pidOutput = -1 * m_controller.calculate(getAbsolutePosition());

    double velocity = m_controller.getSetpoint().velocity;
    double acceleration = (velocity - prevGoalVelocity) / m_controller.getPeriod();

    double feedforwardOutput = -1 * m_feedForward.calculate(getAbsolutePosition(), velocity, acceleration);

    SmartDashboard.putNumber("PID OUTPUT", pidOutput);
    SmartDashboard.putNumber("FEEDFORWARD OUTPUT", feedforwardOutput);
    SmartDashboard.putNumber("SETPOINT", m_controller.getSetpoint().position);
    SmartDashboard.putNumber("GOAL", m_controller.getGoal().position);

    m_pitchControlMotor.set(pidOutput + feedforwardOutput);

    prevGoalVelocity = velocity;

    SmartDashboard.putNumber("ARM MOTOR OUTPUT", pidOutput + feedforwardOutput);
  }
  
  @Override
  public void periodic(){
    SmartDashboard.putNumber("ARM ENCODER POSITION", getAbsolutePosition());
    SmartDashboard.putBoolean("AT GOAL", atGoal());
    SmartDashboard.putNumber("ARM ENCODER RAW", m_encoder.getAbsolutePosition());

    if(goal == 0){
      goal = getAbsolutePosition();
    }
    reachGoal(goal);
  }
}
