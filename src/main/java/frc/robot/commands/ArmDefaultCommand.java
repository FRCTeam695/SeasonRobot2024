// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmDefaultCommand extends Command {
  /** Creates a new armDefaultCommand. */
  ArmSubsystem m_ArmSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  StringSubscriber scoreLocationSub;
  NetworkTable table;
  NetworkTableInstance inst;

  public ArmDefaultCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SideCar");

    m_ArmSubsystem = armSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    
    scoreLocationSub = table.getStringTopic("Score Location").subscribe("");
    //amplifySub = table.getBooleanTopic("Amplify").subscribe(false);
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String scoreLocation = scoreLocationSub.get();
    if(!m_IntakeSubsystem.getNoteStatus()){
      m_ArmSubsystem.setGoal(Constants.Arm.INTAKE_POSITION_RADIANS); //replace with intake position
    }else {
      switch (scoreLocation){
        case "amp":
          m_ArmSubsystem.setGoal(Constants.Arm.AMP_SCORE_RADIANS);//replace with amp position
          break;
        case "speaker":
          m_ArmSubsystem.setGoal(Constants.Arm.STOCKPILE_POSITION_RADIANS);//replace with stockpile position
          break;
        case "intake":
          m_ArmSubsystem.setGoal(Constants.Arm.INTAKE_POSITION_RADIANS);//replace with intake position
          break;
        default:
          break;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
