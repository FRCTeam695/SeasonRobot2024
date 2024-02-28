// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
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
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();

  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private final XboxController controller = new XboxController(0);
  private final XboxController climberController = new XboxController(1);
  private final JoystickButton back_Button = new JoystickButton(controller, 7);
  private final JoystickButton a_Button = new JoystickButton(controller, 1);
  private final JoystickButton b_Button = new JoystickButton(controller, 2);
  private final JoystickButton x_Button = new JoystickButton(controller, 3);

  private final JoystickButton y_Button = new JoystickButton(controller, 4);
  private final JoystickButton left_Bumper = new JoystickButton(controller, 5);
  private final JoystickButton right_Bumper = new JoystickButton(controller, 6);
  private final JoystickButton amplify_Button = new JoystickButton(climberController, 1);

  //Network Table Stuff
  // private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  // private NetworkTable table = inst.getTable("SideCar");
  // private StringSubscriber scoreLocationSub = NetworkTableInstance.getDefault().getStringTopic("Score Location").subscribe("");

  // Double suppliers
  private final DoubleSupplier left_xAxis = () -> (controller.getRawAxis(0));
  private final DoubleSupplier left_yAxis = () -> (controller.getRawAxis(1));
  private final DoubleSupplier right_xAxis = () -> (controller.getRawAxis(4));
  private final DoubleSupplier right_Trigger = () -> (controller.getRawAxis(3));
  private final DoubleSupplier climberController_y1 = () -> MathUtil.applyDeadband(climberController.getRawAxis(1), 0.15);
  private final DoubleSupplier climberController_y2 = ()-> MathUtil.applyDeadband(climberController.getRawAxis(5), 0.15);

  private PIDController xController = new PIDController(1, 0, 0);
  private PIDController yController = new PIDController(1, 0, 0);
  private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Swerve.PROFILED_KP_VALUE, 0,
      0, Constants.Swerve.TRAPEZOID_THETA_CONSTRAINTS);



  // Triggers
  private final Trigger shoot_Trigger = new Trigger(()-> (right_Trigger.getAsDouble() > .60));

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Integer> offsetChooser = new SendableChooser<>();

  public RobotContainer() {

    NamedCommands.registerCommand("Intake Note", intake());
    NamedCommands.registerCommand("Shoot Note", shoot(2222).withTimeout(2));
    NamedCommands.registerCommand("Shoot Position", armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS));

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    defaultCommands();

    xController.reset();
    yController.reset();
    thetaController.reset(null);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Swerve Subsystem", m_SwerveSubsystem);
    SmartDashboard.putData("Intake Subsystem", m_IntakeSubsystem);
    SmartDashboard.putData("Shooter Subsystem", m_ShooterSubsystem);
    
    
    offsetChooser.addOption("left", -120);
    offsetChooser.addOption("right", 120);
    offsetChooser.addOption("mid", 180);
    offsetChooser.setDefaultOption("mid", 180);

    SmartDashboard.putData("gyro offsets", offsetChooser);
  }

  private void configureBindings() {
    //b_Button.onTrue(sausageNote());
    // right_Bumper.onTrue(
    //   sausageToBeamBreak()
    //   .andThen(
    //   flickToAmp())
    // );
    //left_Bumper.onTrue(sausageToBeamBreak());
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
      shootNoteCommand()
      .andThen(armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS)));

    
    /*
     * B BUTTON BINDING:
     * 
     * Resets the gyro to the value chosen on smartdashboard,
     * should be used at the beginning of a match
     */  
    b_Button.onTrue(new InstantCommand(()-> setGyroOffset(), m_SwerveSubsystem).ignoringDisable(true));



    /*
     * RIGHT BUMPER BINDING:
     * 
     * Spits out a note using the intake and indexer
     */
    right_Bumper.whileTrue(
      armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS)
      .andThen(parallel(
        m_IntakeSubsystem.runIndexerToSpeed(1),
        m_ShooterSubsystem.runVelocity(()-> 1000)
      ))

    );


    /*
     * A BUTTON BINDING:
     * 
     * Sausages note then moves the arm to AMP scoring position,
     * then tells the shooter that next time we shoot, to run the shooters at AMP speed.
     */
    a_Button.onTrue(sausageNote()
                    .andThen(armToPosition(Constants.Arm.AMP_SCORE_RADIANS))
                    .andThen(m_ShooterSubsystem.setScoringStatus("amp")));

    //right_Bumper.onTrue(armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS));

    /*
     * X BUTTON BINDING:
     * 
     * Moves the arm to stockpile position,
     * then tells the shooter that next time we shoot, to run the shooters at max speed for stockpile.
     */
    x_Button.onTrue(armToPosition(Constants.Arm.STOCKPILE_POSITION_RADIANS)
                    .andThen(m_ShooterSubsystem.setScoringStatus("stockpile")));

    /*
     * BACK BUTTON BINDING:
     * 
     * Resets the gyro, this is used mostly in testing and 
     * shouldn't really be needed in a match unless something unexpected happens
     */
    back_Button.onTrue(m_SwerveSubsystem.resetSwerve());
    
    /*
     * AMPLIFY BUTTON BINDING:
     * 
     * Gives an LED signal to amplify the speaker
     * If we have already given the signal, if pressed again will turn the signal off
     */
    amplify_Button.onTrue(amplifyButtonCommand());

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
     * 
     * We use closed loop control instead of run velocity because it while fight better,
     * keeps the note in place nicely
     */
    m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.shooterDefaultCommand(0));
    //m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.runVelocity(()-> 0));

    /*
     * Default command for the intake, this ensures that
     * while the intake is not in use it is not running
     */
    m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.runIntakeAndIndexerPercent(0.0));

    /*
     * Default command for the climber, 
     * it reads the y-axis from each controller and sets each climber to speed
     */
    m_ClimberSubsystem.setDefaultCommand(climberDefaultComand());
  }

  /*
   * If we are already amplified, de amplify
   * If we are not amplified, amplify
   */
  private Command amplifyButtonCommand(){
    return new ConditionalCommand(
      
      // Command on true
      m_LedSubsystem.deAmplify(), 

      // Command on false
      m_LedSubsystem.amplifyLED(), 

      // Boolean supplier condition
      ()-> m_LedSubsystem.getColor().equals("amplify"));
  }

  /*
   * continuously runs each climber to a set speed
   */
  private Command climberDefaultComand(){
    return
      m_ClimberSubsystem.runClimberMotors(climberController_y1, climberController_y2);
  }

  /*
   * Runs the intake in while stopping the shooters,
   * it is only organized in a parallel race group as
   * the command would never end if we didn't
   * (runVelocity would just keep going)
   */
  private Command intake() {
    return 

    // makes sure arm is in the correct position
    armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS)
    .andThen(
    
    // (runs the shooter to 0 velocity and run intake and indexer) until we get a beambreak
    race(
      race(
        // LED is for driver preference, so he knows when the robot is still intaking
        m_LedSubsystem.setColorToGreen()
        .andThen(m_IntakeSubsystem.runIntakeAndIndexerPercent(0.5))
        ,
        waitUntil(()-> m_IntakeSubsystem.getBeamBreak())

      ),
      m_ShooterSubsystem.runVelocity(()-> 0))
      )

    // LED to orange means driver can drive away, won't effect the intake, we have the note
    .andThen(m_LedSubsystem.setColorToOrange())

    // runs the indexer backwards using closed loop control,
    // we do this because the shooter wheels need room to spin up to speed
    .andThen(
        race(
          m_IntakeSubsystem.indexerClosedLoopControl(0.2, 0.25),
          waitUntil(()-> !m_IntakeSubsystem.getBeamBreak())
        )
      )
      .andThen(m_IntakeSubsystem.setNoteStatus(true))

      // moves the arm to shoot position, 
      // we default to shoot position until told otherwise by driver
      .andThen(armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS))
      .andThen(m_ShooterSubsystem.setScoringStatus("speaker"))

      // the withName is for names commands
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
  private Command shoot(int percentRPM) {
    return race(
      
      // run the shooter wheels to speed
      m_ShooterSubsystem.runVelocity(()-> (percentRPM/5700.0)),

      // wait for shooters to get up to speed
      waitUntil(m_ShooterSubsystem::isRunning)
        .andThen(waitUntil(m_ShooterSubsystem::shooterIsUpToSpeed))
        .andThen(

            race(

              // the shooters are now up to speed, so we run the indexers forward full speed
              m_IntakeSubsystem.runIndexerToSpeed(1),

              // the following logic watches for a velocity dip in the shooter wheels
              waitUntil(m_ShooterSubsystem::shooterIsNotUpToSpeed)
                .andThen(
                  waitUntil(m_ShooterSubsystem::shooterIsUpToSpeed)
                      )
                )
        )
            
    )

    // tells us that we do not have a note in possesion anymore
    .andThen(m_IntakeSubsystem.setNoteStatus(false))

    // turns the led's off, this tells the driver we do not have a note anymore
    .andThen(m_LedSubsystem.turnColorOff())
    ;
  }

  /*
   * Sausages the note in the shooter wheels, gets us ready for amp scoring
   */
  private Command sausageNote(){
      return 
          // Uses closed loop control in indexer to move the note into the shooter wheel
          m_IntakeSubsystem.indexerClosedLoopControl(1.5, 1)

          // Moves the note into place by using closed loop control
          .andThen(
            race(
              // ends once it reaches the desired amount of rotations (0.75 in this case)
              m_ShooterSubsystem.closedLoopRotation(0.75, 1, 0.0),

              // Doesn't end
              m_IntakeSubsystem.runIndexerToSpeed(0.1)
            )
          );
  }
  
  /*
   * Gives a quick closed loop rotation to give the note a quick push into the amp
   */

  private Command flickToAmp(){

    // kp will most likely be subject to tuning
    return m_ShooterSubsystem.closedLoopRotation(1.1, 0.17, 0.0);

  }

  private Command sausageToBeamBreak(){
    return m_ShooterSubsystem.runVelocity(()-> 0.025).until(()-> m_IntakeSubsystem.getBeamBreak())
            .andThen(m_ShooterSubsystem.closedLoopRotation(0, 0, 0));
  }

  /*
   * Moves the arm to a given position
   * 
   * @param position , a position in radians for the arm to move to 
   */
  private Command armToPosition(double position) {
    return new FunctionalCommand(
      // We have to reset state to present or else the PID controller 
      // doesn't always have an accurate reading of where it is

      // ** INIT **
      m_ArmSubsystem::resetStateToPresent,

      // ** EXECUTE **
      ()-> m_ArmSubsystem.setGoal(position),

      // ** ON INTERRUPTED **
      interrupted->{},

      // ** END CONDITION **
      m_ArmSubsystem::atGoal,

      // ** REQUIREMENTS **
      m_ArmSubsystem)

      // withName is just for named commands
      .withName("Shoot Position");
  }

  /*
   * Shoot the note, the same button is binded to amp and speaker this way
   */
  private Command shootNoteCommand(){
    return new SelectCommand<>(

      Map.ofEntries(

        Map.entry("amp", 

          // Moves arm to position
          armToPosition(Constants.Arm.AMP_SCORE_RADIANS)

          // Flicks the note into the amp
          .andThen(sausageToBeamBreak())
          .andThen(
          flickToAmp()

          // The wait command is here because if we move the arm back to intake position 
          //immediatly after amp scoring, if we miss the note will bounce back and get stuck in the robot
          .andThen(m_LedSubsystem.turnColorOff())
          .andThen(new WaitCommand(1.0)))),

          
        Map.entry("speaker", 

          // Moves the arm into position for shooting
          armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS)

          // Shoots the note into speaker
          .andThen(shoot(2222)).withTimeout(2)
          .andThen(m_LedSubsystem.turnColorOff())),


        Map.entry("stockpile", 

          // Moves arm into position for stockpiling
          armToPosition(Constants.Arm.STOCKPILE_POSITION_RADIANS)

          // Shoots the note at full speed
          .andThen(shoot(2222)).withTimeout(2))
          
      ),

    // condition (boolean supplier)
    ()-> m_ShooterSubsystem.getScoringStatus());
  }

  public Command scorePreload(){
    return armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS).andThen(shoot(2222));
  }

  public void setGyroOffset(){
    double val = offsetChooser.getSelected();
    m_SwerveSubsystem.setGyro(val);
  }

  // public Command startingAuton(){
  //   //return scorePreload()
  //   //.andThen(mid_to_2_command())
  //   return mid_to_2_command()
  //   ;
  // }

  // public Command mid_to_2_command(){
  //   Trajectory trajectory1 = mid_to_2();

  //   var swerveControllerCommand = new SwerveControllerCommand(trajectory1, m_SwerveSubsystem::getPose,
  //       Constants.Swerve.kDriveKinematics, xController, yController, thetaController, m_SwerveSubsystem::setModules,
  //       m_SwerveSubsystem);

  //   return runOnce(
  //     ()-> m_SwerveSubsystem.resetOdometry(trajectory1.getInitialPose())
  //   )
  //   .andThen(swerveControllerCommand)
  //   .andThen(()-> m_SwerveSubsystem.stopModules())
  //   ;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return startingAuton();
    return autoChooser.getSelected();
    //return new WaitCommand(1);
  }

  // public Trajectory mid_to_2(){
  //   TrajectoryConfig trajectoryConfig1 = new TrajectoryConfig(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS * 0.2, 
  //                                                             3);
    
  //   Trajectory trajectory1 =
  //       TrajectoryGenerator.generateTrajectory(

  //         new Pose2d(0, 0, new Rotation2d(Math.PI)),

  //         List.of(new Translation2d(Units.inchesToMeters(30), 0)),

  //         new Pose2d(Units.inchesToMeters(62), Units.inchesToMeters(0), new Rotation2d(Math.PI)),

  //         trajectoryConfig1);

  //   return trajectory1;
  // }






}
