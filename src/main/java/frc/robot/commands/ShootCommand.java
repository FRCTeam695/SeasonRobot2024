// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  ShooterSubsystem m_ShooterSubsubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  /** Creates a new ShootCommand. */
  DoubleSupplier velocity1;
  public ShootCommand(ShooterSubsystem m_ShooterSubsubsystem, IntakeSubsystem m_IntakeSubsystem) {
    this.m_ShooterSubsubsystem = m_ShooterSubsubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    velocity1 = () -> (100);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsubsystem.runVelocity(velocity1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_ShooterSubsubsystem.shooterIsUpToSpeed() ){
      //shoot
      m_IntakeSubsystem.runIndexerToSpeed(1);
      //ADD A DELAY SOMEHOW
      m_IntakeSubsystem.setNoteStatus(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.runIndexerToSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_IntakeSubsystem.getNoteStatus();
  }
}
