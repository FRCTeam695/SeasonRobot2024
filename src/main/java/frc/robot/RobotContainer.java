// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private final XboxController controller = new XboxController(0);
  private final JoystickButton back_Button = new JoystickButton(controller, 7);
  private final JoystickButton a_Button = new JoystickButton(controller, 1);
  private final JoystickButton y_Button = new JoystickButton(controller, 4);
  private final JoystickButton left_Bumper = new JoystickButton(controller, 5);
  private final JoystickButton right_Bumper = new JoystickButton(controller, 6);

  //Network Table Subscribers
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("SideCar");
  StringSubscriber scoreLocationSub = NetworkTableInstance.getDefault().getStringTopic("Score Location").subscribe("");

  // Double suppliers
  private final DoubleSupplier left_xAxis = () -> (controller.getRawAxis(0));
  private final DoubleSupplier left_yAxis = () -> (controller.getRawAxis(1));
  private final DoubleSupplier right_xAxis = () -> (controller.getRawAxis(4));
  private final DoubleSupplier right_Trigger = () -> (controller.getRawAxis(3));


  // Triggers
  private final Trigger shoot_Trigger = new Trigger(()-> (right_Trigger.getAsDouble() > .60));

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("Intake Note", intake());
    NamedCommands.registerCommand("Shoot Note", shoot().andThen(runOnce(()-> m_LedSubsystem.turnColorOff())));
    NamedCommands.registerCommand("Shoot Position", 
    runOnce(
      ()-> m_ArmSubsystem.setGoal(1.05), m_ArmSubsystem
    )
    .andThen(waitUntil(()-> m_ArmSubsystem.atGoal()))
    );

    //NamedCommands.registerCommand("Intake Note", new PrintCommand("intake"));
    //NamedCommands.registerCommand("Shoot Note", new PrintCommand("shoot"));

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    instantCommands();
    defaultCommands();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("theta", 0.99);
  }

  private void configureBindings() {
    // goes to a location
    //x_Button.onTrue(m_swerveSubsystem.goToLocation(new Pose2d(3, 2.5, new Rotation2d(0))));

    /*
     * LEFT BUMPER BINDING:
     * 
     * Intakes a note, runs the intake and the indexer
     */
    left_Bumper.onTrue(
      intake());


    /*
     * "RIGHT TRIGGER" BUTTON BINDING:
     * 
     * Shoots the note, this involves spinning the shooter up to 
     * speed and then feeding the note in with the indexer,
     * stops after two seconds.
     */
    shoot_Trigger.onTrue(
      shoot().andThen(runOnce(()-> m_LedSubsystem.turnColorOff()))
    );



    /*
     * RIGHT BUMPER BINDING:
     * 
     * Spits out a note using the intake and indexer
     */
    //right_Bumper.whileTrue(run(()-> m_intakeSubsystem.runIntakeAndIndexerPercent(-0.1), m_intakeSubsystem));

    //stockpile
    right_Bumper.onTrue(runOnce(()-> m_ArmSubsystem.setGoal(Constants.Arm.STOCKPILE_POSITION_RADIANS), m_ArmSubsystem));

    //Intake angle
    a_Button.onTrue(runOnce(()-> m_ArmSubsystem.setGoal(SmartDashboard.getNumber("theta", 0.19)), m_ArmSubsystem));

    //Stockpile
    //left_Bumper.whileTrue(m_ArmSubsystem.goToHeight(0.14));

    // This button was only used for testing purposes
    //indexButton11.onTrue(intake().andThen(shoot().withTimeout(2)));
  }

  private void instantCommands() {

    /*
     * BACK BUTTON BINDING:
     * 
     * Resets the gyro, this is used mostly in testing and 
     * shouldn't really be needed in a match unless something unexpected happens
     */
    back_Button.onTrue(runOnce(()-> m_swerveSubsystem.zeroHeading()));

    
    // This is just for testing purposes
    y_Button.onTrue(runOnce(()-> m_swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));

  }

  private void defaultCommands() {
    /*
     * Default command for drive train (swerve)
     */
    m_swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_swerveSubsystem, left_xAxis, left_yAxis, right_xAxis, true));

    /*
     * Default command for shooter, this ensures that 
     * while the shooter is not in use it is not running
     */
    m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.runVelocity(()-> 0));

    /*
     * Default command for the intake, this ensures that
     * while the intake is not in use it is not running
     */
    m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> {m_intakeSubsystem.runIntakeAndIndexerPercent(0.0);}, m_intakeSubsystem));

    /*
     * Default command for arm, this checks if we have a note or not,
     * and then moves the arm to the according position
    
    m_ArmSubsystem.setDefaultCommand(
      race(
        waitUntil(()-> m_intakeSubsystem.getNoteStatus()),
        m_ArmSubsystem.goToAngle(Constants.Arm.INTAKE_POSITION_RADIANS)
      )
      .andThen(
        race(
          waitUntil(()-> !m_intakeSubsystem.getNoteStatus()),
          m_ArmSubsystem.goToAngle(Constants.Arm.STOCKPILE_POSITION_RADIANS) //The shooting position
        )
      )
    );
    */
    
  }

  /*
   * Runs the intake in while stopping the shooters,
   * it is only organized in a parallel race group as
   * the command would never end if we didn't
   * (runVelocity would just keep going)
   */
  private Command intake() {
    return 
    runOnce(
      ()-> m_ArmSubsystem.setGoal(Constants.Arm.INTAKE_POSITION_RADIANS), m_ArmSubsystem
    )
    .andThen(waitUntil(()-> m_ArmSubsystem.atGoal()))
    .andThen(
    race(
      race(
        runOnce(()-> m_LedSubsystem.setColorToGreen(), m_intakeSubsystem)
          .andThen(
            run(()-> m_intakeSubsystem.runIntakeAndIndexerPercent(0.5), m_intakeSubsystem))
        , waitUntil(()-> m_intakeSubsystem.getBeamBreak())
      ),
      m_ShooterSubsystem.runVelocity(()-> 0)))
      .andThen(runOnce(()-> m_LedSubsystem.setColorToOrange()))
      .andThen(
          race(
            run(()-> m_intakeSubsystem.runIndexerToSpeed(-0.05)),
            waitUntil(()-> !m_intakeSubsystem.getBeamBreak())
          )
      )
      .andThen(runOnce(()-> m_intakeSubsystem.setNoteStatus(true), m_intakeSubsystem))
      ;
  }

  /*
   * Runs the shooters up to speed,
   * when they are up to speed the indexer 
   * will feed a note in full speed into the shooter
   * 
   * The shooter will turn off automatically by watching the 
   * velocity time graph for the shootermotor for a dip (indicating we shot the note)
   */
  private Command shoot() {
    return race(
      m_ShooterSubsystem.runVelocity(()-> (2221.0/5700.0)),
      waitUntil(m_ShooterSubsystem::isRunning)
        .andThen(waitUntil(m_ShooterSubsystem::shooterIsUpToSpeed))
        .andThen(
            race(
              run(()-> m_intakeSubsystem.runIndexerToSpeed(1)),
              waitUntil(m_ShooterSubsystem::shooterIsNotUpToSpeed)
                .andThen(
                  waitUntil(m_ShooterSubsystem::shooterIsUpToSpeed)
                      )
                )
        )
            
    )
    .andThen(runOnce(()-> m_intakeSubsystem.setNoteStatus(false)))
    ;
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
