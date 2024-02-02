// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  DigitalInput beamBreak;
  int speed;
  int stage;
  IntakeSubsystem m_IntakeSubsystem;
  boolean done = false;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem sub) {
    m_IntakeSubsystem = sub;
    speed = 0;
    stage = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage == 1) { // stage 1 is jamming the note into the shooter motors until the beambreak is
                      // broken
      if (m_IntakeSubsystem.getBeamBreak()) { // if beambreak is broken
        speed = 0;
        stage = 2;
        m_IntakeSubsystem.runSubsystemToSpeed(0);
      } else {
        m_IntakeSubsystem.runSubsystemToSpeed(speed);
      }
    } else if (stage == 2) { // stage 2 is rocking back the indexer until the beam is unbroken
      if (!beamBreak.get()) { // if beambreak is unbroken
        stage = 1;
        m_IntakeSubsystem.setNoteStatus(true);
        done = true;
      } else {
        m_IntakeSubsystem.runIndexerToSpeed(-0.1);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.runSubsystemToSpeed(speed);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
