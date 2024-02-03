// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final XboxController controller = new XboxController(0);
  private final Joystick shootStick = new Joystick(1);
  private final Joystick intakeStick = new Joystick(2);
  private final JoystickButton back_Button = new JoystickButton(controller, 7);
  private final JoystickButton a_Button = new JoystickButton(controller, 1);
  private final JoystickButton x_Button = new JoystickButton(controller, 3);
  private final JoystickButton y_Button = new JoystickButton(controller, 4);
  private final JoystickButton left_Bumper = new JoystickButton(controller, 5);
  private final JoystickButton right_Bumper = new JoystickButton(controller, 6);
  private final JoystickButton shoot_Button = new JoystickButton(shootStick, 1);
  private final JoystickButton intake_Button = new JoystickButton(intakeStick, 1);
  private final JoystickButton indexButton11 = new JoystickButton(intakeStick, 11);
  

  private final DoubleSupplier left_xAxis = () -> (controller.getRawAxis(0));
  private final DoubleSupplier left_yAxis = () -> (controller.getRawAxis(1));
  private final DoubleSupplier right_xAxis = () -> (controller.getRawAxis(4));
  private final DoubleSupplier shootStickAdjuster = () -> 1 - ((shootStick.getRawAxis(3)) + 1)/2.0;

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

    SmartDashboard.putData("Intake Subsytem", m_intakeSubsystem);
    SmartDashboard.putData("Shooter Subsytem", m_ShooterSubsystem);
  }

  private void configureBindings() {
    // goes to a location
    //x_Button.onTrue(m_swerveSubsystem.goToLocation(new Pose2d(3, 2.5, new Rotation2d(0))));

    // intakes a note (this includes running the indexer)
    left_Bumper.onTrue(
      intake());

    a_Button.onTrue(
      shoot().withTimeout(2)
    );

    indexButton11.onTrue(intake().andThen(shoot().withTimeout(2)));


    right_Bumper.whileTrue(run(()-> m_intakeSubsystem.runIntakeAndIndexerPercent(-0.1), m_intakeSubsystem));
  }

  private Command intake() {
    return race(
      new IntakeCommand(m_intakeSubsystem, 0.3), 
      m_ShooterSubsystem.runVelocity(()-> 0));
  }

  private Command shoot() {
    return parallel(
      m_ShooterSubsystem.runVelocity(()-> (2221.0/5700.0)),
      waitUntil(m_ShooterSubsystem::isRunning)
        .andThen(waitUntil(m_ShooterSubsystem::shooterIsUpToSpeed))
        .andThen(run(()-> m_intakeSubsystem.runIndexerToSpeed(1))));
  }

  private void instantCommands() {

    // zeroes heading on gyro
    back_Button.onTrue(new InstantCommand(() -> {
      m_swerveSubsystem.zeroHeading();
    }, m_swerveSubsystem));

    // resets odometry
    y_Button.onTrue(new InstantCommand(() -> {
      m_swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(70)));
    }, m_swerveSubsystem));

  }

  private void defaultCommands() {
    m_swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_swerveSubsystem, left_xAxis, left_yAxis, right_xAxis, true));
    m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.runVelocity(()-> 0));
    m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> {m_intakeSubsystem.runIntakeAndIndexerPercent(0.0);}, m_intakeSubsystem));
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
