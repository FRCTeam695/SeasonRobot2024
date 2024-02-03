// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  double speed;
  double initialSpeed;
  int stage;
  IntakeSubsystem m_IntakeSubsystem;
  boolean done = false;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem sub,double speed) {
    m_IntakeSubsystem = sub;
    this.initialSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 1;
    speed = initialSpeed;
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage == 1) { // stage 1 is jamming the note into the shooter motors until the beambreak is
                      // broken
      if (m_IntakeSubsystem.getBeamBreak()) { // if beambreak is broken
        speed = 0;
        stage = 2;
        m_IntakeSubsystem.runIntakeAndIndexerPercent(0);
      } else {
        m_IntakeSubsystem.runIntakeAndIndexerPercent(speed);
      }
    } 
    else if (stage == 2) { // stage 2 is rocking back the indexer until the beam is unbroken
      if (!m_IntakeSubsystem.getBeamBreak()) { // if beambreak is unbroken
        stage = 1;
        m_IntakeSubsystem.setNoteStatus(true);
        done = true;
      } else {
        m_IntakeSubsystem.runIndexerToSpeed(-0.05);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.runIntakeAndIndexerPercent(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
