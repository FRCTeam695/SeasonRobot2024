// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmDefaultCommand;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
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

  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();

  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private final XboxController controller = new XboxController(0);
  private final JoystickButton back_Button = new JoystickButton(controller, 7);
  private final JoystickButton a_Button = new JoystickButton(controller, 1);
  private final JoystickButton b_Button = new JoystickButton(controller, 2);
    private final JoystickButton x_Button = new JoystickButton(controller, 3);

  private final JoystickButton y_Button = new JoystickButton(controller, 4);
  private final JoystickButton left_Bumper = new JoystickButton(controller, 5);
  private final JoystickButton right_Bumper = new JoystickButton(controller, 6);

  //Network Table Stuff
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("SideCar");
  private StringSubscriber scoreLocationSub = NetworkTableInstance.getDefault().getStringTopic("Score Location").subscribe("");

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
    NamedCommands.registerCommand("Shoot Note", autonShoot());
    NamedCommands.registerCommand("Shoot Position", armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS));

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    defaultCommands();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Intake Subsystem", m_IntakeSubsystem);
    SmartDashboard.putData("Shooter Subsystem", m_ShooterSubsystem);
    //SmartDashboard.putNumber("theta", 0.99);
  }

  private void configureBindings() {
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
      scoreNoteCommand()
    );



    /*
     * RIGHT BUMPER BINDING:
     * 
     * Spits out a note using the intake and indexer
     */
    //right_Bumper.whileTrue(run(()-> m_intakeSubsystem.runIntakeAndIndexerPercent(-0.1), m_intakeSubsystem));

    a_Button.onTrue(sausageNote()
                    .andThen(armToPosition(Constants.Arm.AMP_SCORE_RADIANS))
                    .andThen(m_ShooterSubsystem.setScoringStatus("amp")));

    right_Bumper.onTrue(armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS));
    x_Button.onTrue(armToPosition(Constants.Arm.STOCKPILE_POSITION_RADIANS));

    /*
     * BACK BUTTON BINDING:
     * 
     * Resets the gyro, this is used mostly in testing and 
     * shouldn't really be needed in a match unless something unexpected happens
     */
    back_Button.onTrue(runOnce(()-> m_SwerveSubsystem.zeroHeading()));

    // This is just for testing purposes
    y_Button.onTrue(runOnce(()-> m_SwerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));
  }

  private void defaultCommands() {
    /*
     * Default command for drive train (swerve)
     */
    m_SwerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_SwerveSubsystem, left_xAxis, left_yAxis, right_xAxis, true));

    /*
     * Default command for shooter, this ensures that 
     * while the shooter is not in use it is not running
     */
    m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.runVelocity(()-> 0));

    /*
     * Default command for the intake, this ensures that
     * while the intake is not in use it is not running
     */
    m_IntakeSubsystem.setDefaultCommand(new RunCommand(() -> {m_IntakeSubsystem.runIntakeAndIndexerPercent(0.0);}, m_IntakeSubsystem));

    /*
     * Default command for arm, this checks if we have a note or not,
     * and then moves the arm to a position as specified in sidecar
     */
    //m_ArmSubsystem.setDefaultCommand(new ArmDefaultCommand(m_ArmSubsystem, m_IntakeSubsystem));  
  }

  /*
   * Runs the intake in while stopping the shooters,
   * it is only organized in a parallel race group as
   * the command would never end if we didn't
   * (runVelocity would just keep going)
   */
  private Command intake() {
    return 
    armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS)
    .andThen(
    race(
      race(
        runOnce(()-> m_LedSubsystem.setColorToGreen())
          .andThen(
            run(()-> m_IntakeSubsystem.runIntakeAndIndexerPercent(0.5), m_IntakeSubsystem))
        , waitUntil(()-> m_IntakeSubsystem.getBeamBreak())
      ),
      m_ShooterSubsystem.runVelocity(()-> 0))
      )
      .andThen(runOnce(()-> m_LedSubsystem.setColorToOrange()))
      .andThen(
          race(
            //run(()-> m_IntakeSubsystem.runIndexerToSpeed(-0.05)),
            m_IntakeSubsystem.indexerClosedLoopControl(0.2, 0.25),
            waitUntil(()-> !m_IntakeSubsystem.getBeamBreak())
          )
      )
      .andThen(runOnce(()-> m_IntakeSubsystem.setNoteStatus(true), m_IntakeSubsystem))
      .andThen(armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS))
      .andThen(m_ShooterSubsystem.setScoringStatus("speaker"))
      .withName("Intake Note")
      ;
  }

  /*
   * Runs the shooters up to speed,
   * when they are up to speed the indexer 
   * will feed a note in full speed into the shooter
   * 
   * The shooter will turn off automatically by watching the 
   * velocity time graph for the shootermotor for a dip (indicating we shot the note)
   * or with just a two second timeout (just in case it somehow shoots without having a note)
   */
  private Command shoot() {
    return race(
      m_ShooterSubsystem.runVelocity(()-> (2222/5700.0)),//2221.0 for speaker; 550 for amp
      waitUntil(m_ShooterSubsystem::isRunning)
        .andThen(waitUntil(m_ShooterSubsystem::shooterIsUpToSpeed))
        .andThen(
            race(
              run(()-> m_IntakeSubsystem.runIndexerToSpeed(1), m_IntakeSubsystem),
              waitUntil(m_ShooterSubsystem::shooterIsNotUpToSpeed)
                .andThen(
                  waitUntil(m_ShooterSubsystem::shooterIsUpToSpeed)
                      )
                )
        )
            
    )
    .andThen(runOnce(()-> m_IntakeSubsystem.setNoteStatus(false)))
    .andThen(runOnce(()-> m_LedSubsystem.turnColorOff()))
    //.andThen(m_ShooterSubsystem.runVelocity(()-> (550.0 / 5700.0)).withTimeout(2)) //amp shooting follow through
    ;
  }

  private Command autonShoot(){
    return shoot().andThen(runOnce(()-> m_LedSubsystem.turnColorOff())).withName("Shoot Note");
  }

  /*
   * Sausages the note in the shooter wheels, gets us ready for amp scoring
   */
  private Command sausageNote(){
        return m_IntakeSubsystem.indexerClosedLoopControl(0.75, 2)
                .andThen(m_ShooterSubsystem.closedLoopRotation(0.75, 1));
  }

  /*
   * Gives a quick closed loop rotation to give the note a quick push into the amp
   */
  private Command flickToAmp(){
    return m_ShooterSubsystem.closedLoopRotation(2, 0.6);
  }

  /*
   * Moves the arm to a given position
   * 
   * @param position , a position in radians for the arm to move to 
   */
  private Command armToPosition(double position) {
    return new FunctionalCommand(
      m_ArmSubsystem::resetStateToPresent,
      ()-> m_ArmSubsystem.setGoal(position),
      interrupted->{},
      m_ArmSubsystem::atGoal,
      m_ArmSubsystem)
      .withName("Shoot Position");
  }

  /*
   * Scores the note, the same button is binded to amp and speaker this way
   */
  private Command scoreNoteCommand(){
    return new ConditionalCommand(

    armToPosition(Constants.Arm.AMP_SCORE_RADIANS)
    .andThen(flickToAmp())
    .andThen(armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS)), 

    armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS)
    .andThen(shoot()).withTimeout(2)
    .andThen(armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS)), 

    ()-> m_ShooterSubsystem.getScoringStatus().equals("amp"));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
    //return testAuton();
  }

  public Command testAuton(){
    return armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS).andThen(autonShoot());
  }
}
