// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final SwerveSubsystem m_Subsystem;
  private final DoubleSupplier xSpeed, ySpeed, turningSpeed;
  private final boolean fieldOriented;


  public SwerveDriveCommand(SwerveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier turningSpeed, boolean fieldOriented) {
    this.m_Subsystem = subsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.turningSpeed = turningSpeed;
    this.fieldOriented = fieldOriented;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(DriverStation.isAutonomous()){
      return;
    }
    
    //Gets values from double suppliers
    Double Xj = -1 * xSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below
    Double Yj = -1 * ySpeed.getAsDouble(); //The controller is inverted
    Double Zj = -1 * turningSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below

    if(m_Subsystem.isFlipped()){
      Xj *= -1;
      Yj *= -1;
    }

    // JPK:  scaled deadband
    double db = 0.2;
    Xj = MathUtil.applyDeadband(Xj, db);
    Yj = MathUtil.applyDeadband(Yj, db);
    Zj = MathUtil.applyDeadband(Zj, db);

    SmartDashboard.putNumber("Zj", Zj);
    SmartDashboard.putNumber("Xj", Xj);
    SmartDashboard.putNumber("Yj", Yj);

    // scale up the speeds, WPILib likes them in meters per second
    Xj = Xj * Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
    Yj = Yj * Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
    Zj = Zj * Constants.Swerve.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

    // construct chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(Yj, Xj, Zj, m_Subsystem.getGyroHeading());
    } else {
        chassisSpeeds = new ChassisSpeeds(Yj, Xj, Zj);
    }

    m_Subsystem.driveSwerve(chassisSpeeds);
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
