// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final XboxController controller = new XboxController(0);
  private final JoystickButton back_Button = new JoystickButton(controller, 7);
  private final JoystickButton a_Button = new JoystickButton(controller, 1);
  private final JoystickButton x_Button = new JoystickButton(controller, 3);
  private final JoystickButton y_Button = new JoystickButton(controller, 4);
  private final JoystickButton left_Bumper = new JoystickButton(controller, 5);
  private final JoystickButton right_Bumper = new JoystickButton(controller, 6);

  private final DoubleSupplier left_xAxis = () -> (controller.getRawAxis(0));
  private final DoubleSupplier left_yAxis = () -> (controller.getRawAxis(1));
  private final DoubleSupplier right_xAxis = () -> (controller.getRawAxis(4));

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("Raise Intake", new PrintCommand("Raising The Intake!"));
    NamedCommands.registerCommand("Lower Intake", new PrintCommand("Lowering The Intake!"));
    NamedCommands.registerCommand("Score Cube", new PrintCommand("Scoring The Cube!"));

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    instantCommands();
    defaultCommands();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Intake Speed", 0.5);
  }

  private void configureBindings() {
    // goes to a location
    x_Button.onTrue(m_swerveSubsystem.goToLocation(new Pose2d(3, 2.5, new Rotation2d(0))));
  }

  private void instantCommands() {

    // zeroes heading on gyro
    back_Button.onTrue(new InstantCommand(() -> {
      m_swerveSubsystem.zeroHeading();
    }, m_swerveSubsystem));

    // intakes a note (this includes running the indexer)
    right_Bumper.whileTrue(new InstantCommand(() -> {
      m_intakeSubsystem.setSpeed(0.3);
    }, m_intakeSubsystem));

    // turns off intake when button is let go
    right_Bumper.onFalse(new InstantCommand(() -> {
      m_intakeSubsystem.setSpeed(0);
    }, m_intakeSubsystem));

    // discharges a note
    left_Bumper.whileTrue(new InstantCommand(() -> {
      m_intakeSubsystem.setSpeed(-0.1);
    }, m_intakeSubsystem));

    // turns off intake when button is let go
    left_Bumper.onFalse(new InstantCommand(() -> {
      m_intakeSubsystem.setSpeed(0);
    }, m_intakeSubsystem));

    // shoots note
    a_Button.onTrue(new InstantCommand(() -> {
      m_intakeSubsystem.shoot();
    }, m_intakeSubsystem));

    // resets odometrys
    y_Button.onTrue(new InstantCommand(() -> {
      m_swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(70)));
    }, m_swerveSubsystem));

  }

  private void defaultCommands() {
    m_swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_swerveSubsystem, left_xAxis, left_yAxis, right_xAxis, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}
