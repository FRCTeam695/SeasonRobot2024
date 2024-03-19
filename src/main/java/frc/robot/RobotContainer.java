// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringLocation;
import frc.robot.subsystems.ShooterSubsystem; 
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
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

  private final SwerveSubsystem Swerve = new SwerveSubsystem();
  private final IntakeSubsystem Intake = new IntakeSubsystem();
  private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final ClimberSubsystem Climber = new ClimberSubsystem();
  private final LEDSubsystem LEDs = new LEDSubsystem();
  private final ArmSubsystem Arm = new ArmSubsystem();
  private final AmpBar AmpBar = new AmpBar();

  private final XboxController driver = new XboxController(0);
  private final JoystickButton back_Button = new JoystickButton(driver, 7);
  private final JoystickButton a_Button = new JoystickButton(driver, 1);
  private final JoystickButton b_Button = new JoystickButton(driver, 2);
  private final JoystickButton x_Button = new JoystickButton(driver, 3);
  private final JoystickButton right_Bumper = new JoystickButton(driver, 6);
  private final JoystickButton y_Button = new JoystickButton(driver, 4);
  private final JoystickButton left_Bumper = new JoystickButton(driver, 5);


  private final XboxController operator = new XboxController(1);
  private final JoystickButton y_Button_Climber = new JoystickButton(operator, 4);
  private final JoystickButton right_Bumper_Climber = new JoystickButton(operator, 6);
  private final JoystickButton amplify_Button_Climber = new JoystickButton(operator, 1);

  //Network Table Stuff
  // private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  // private NetworkTable table = inst.getTable("SideCar");
  // private StringSubscriber scoreLocationSub = NetworkTableInstance.getDefault().getStringTopic("Score Location").subscribe("");

  // Double suppliers
  private final DoubleSupplier left_xAxis = () -> (driver.getRawAxis(0));
  private final DoubleSupplier left_yAxis = () -> (driver.getRawAxis(1));
  private final DoubleSupplier right_xAxis = () -> (driver.getRawAxis(4));
  private final DoubleSupplier right_Trigger = () -> (driver.getRawAxis(3));
  private final DoubleSupplier left_Trigger = ()-> (driver.getRawAxis(2));
  
  private final DoubleSupplier right_Trigger_Operator = () -> (operator.getRawAxis(3));
  private final DoubleSupplier climberLeft = () -> MathUtil.applyDeadband(operator.getRawAxis(1), 0.05);
  private final DoubleSupplier climberRight = ()-> MathUtil.applyDeadband(operator.getRawAxis(5), 0.05);


  // Triggers
  private final Trigger shoot_Trigger = new Trigger(()-> (right_Trigger.getAsDouble() > .60));
  private final Trigger special_Shot_Trigger = new Trigger(()-> (right_Trigger.getAsDouble() > 0.60));
  private final Trigger podium_shot_Trigger = new Trigger(() -> (right_Trigger_Operator.getAsDouble() > .60) );

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("Intake Note", intake());
    NamedCommands.registerCommand("Shoot Note", shoot(Constants.Shooter.RPM_SPEAKER).withTimeout(2));
    NamedCommands.registerCommand("Shoot Hard", shoot(4000).withTimeout(2));
    NamedCommands.registerCommand("Podium Shot", shoot(3000).withTimeout(2));
    NamedCommands.registerCommand("Podium Shot Position", armToPosition(Math.toRadians(40)));
    NamedCommands.registerCommand("Shoot Position", armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS));
    NamedCommands.registerCommand("Shoot Hard Position", armToPosition(Units.degreesToRadians(43)));

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    defaultCommands();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Swerve Subsystem", Swerve);
    SmartDashboard.putData("Intake Subsystem", Intake);
    SmartDashboard.putData("Shooter Subsystem", Shooter);
  }

  private void configureBindings() {
     y_Button.onTrue(
      //AmpBar.closedLoopControl(12.1)
       armToPosition(Math.toRadians(42))
       .andThen(shoot(3000))
       .withTimeout(2)
     );

    /*
     * B BUTTON BINDING:
     * 
     * 
     */  
    b_Button.onTrue(AmpBar.closedLoopControl(0));

     /*
    y_Button_Climber.onTrue(armToPosition(Constants.Arm.AMP_SCORE_RADIANS)
                            .andThen(shoot(1000))
                            .withTimeout(2));
    */

    podium_shot_Trigger.onTrue(podiumShotCommand());


    /*
     * LEFT BUMPER BINDING:
     * 
     * Intakes a note, runs the intake and the indexer
     */
    left_Bumper.onTrue
    (
      intake()
    );


    /*
     * "RIGHT TRIGGER" BUTTON BINDING:
     * 
     * Shoots the note, this involves spinning the shooter up to 
     * speed and then feeding the note in with the indexer,
     * stops after two seconds.
     */
    shoot_Trigger.onTrue
    (
      shootNoteCommand()
      
      .andThen
        (
          armToPosition(Constants.Arm.INTAKE_POSITION_RADIANS)
        )
    );

    special_Shot_Trigger.whileTrue
    (
      Swerve.setRotationOverride(true)
      .andThen
      (
        parallel
        (
          armToPosition(Math.toRadians(42)),
          spinShooterToSpeed(3000)
        )
      )
    );

    /*
     * TODO : TUNE THETA CONTROLLER AND FIND THE CORRECT TAG NUMBER FOR BLUE SPEAKER
     */
    special_Shot_Trigger.onFalse
    (
      armToPosition(Math.toRadians(42))

      .andThen
      (
        shoot(3000)
      )

      .andThen
      (
        Swerve.setRotationOverride(false)
      )
    );



    /*
     * RIGHT BUMPER BINDING:
     * 
     * Spits out a note using the intake and indexer
     */
    right_Bumper.whileTrue
    (
      armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS)
      
      .andThen
      (
        parallel(
          Intake.runIndexerToSpeed(1),
          Shooter.runVelocity(()-> 600)
        )
      )

    );


    /*
     * A BUTTON BINDING:
     * 
     * Sausages note then moves the arm to AMP scoring position,
     * then tells the shooter that next time we shoot, to run the shooters at AMP speed.
     */
    a_Button.onTrue(
                    Shooter.setScoringStatus("amp")
                    .andThen
                    (
                      parallel
                      (
                        armToPosition(Constants.Arm.AMP_SCORE_RADIANS),
                        spinShooterToSpeed(1000)
                      )
                    )
                    );


    /*
     * X BUTTON BINDING:
     * 
     * Moves the arm to stockpile position,
     * then tells the shooter that next time we shoot, to run the shooters at max speed for stockpile.
     */
    x_Button.onTrue
    (
      armToPosition(Constants.Arm.STOCKPILE_POSITION_RADIANS)

      .andThen
      (
        Shooter.setScoringStatus("stockpile")
      )
    );

    /*
     * BACK BUTTON BINDING:
     * 
     * Resets the gyro, this is used mostly in testing and 
     * shouldn't really be needed in a match unless something unexpected happens
     */
    back_Button.onTrue
    (
      Swerve.resetSwerve()
    );
    
    /*
     * AMPLIFY BUTTON BINDING:
     * 
     * Gives an LED signal to amplify the speaker
     * If we have already given the signal, if pressed again will turn the signal off
     */
    amplify_Button_Climber.onTrue
    (
      amplifyButtonCommand()
    );

    /*
     * Y BUTTON BINDING:
     * 
     * Moves the arm to shooting positon and sets the scoring status to speaker
     * This is used incase Ben accidentally stockpiles when meaning to shoot.
     */
    /*
    y_Button.onTrue
    (
      armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS)

      .andThen
      (
        Shooter.setScoringStatus("speaker")
      )
    );
    */

    /*
     * CLIMBER CONTROLLER RIGHT BUMPER BINDING:
     * 
     * Allows the operator to override the softlimits on the climbers temporarily
     */
    right_Bumper_Climber.whileTrue
    (
      Climber.softLimitOverrideSlow(climberLeft, climberRight)
    );
  }


  private void defaultCommands() {
    AmpBar.setDefaultCommand(AmpBar.closedLoopControl(0));
    /*
     * Default command for drive train (swerve)
     */
    Swerve.setDefaultCommand(new SwerveDriveCommand(Swerve, left_xAxis, left_yAxis, right_xAxis, true));

    /*
     * Default command for shoot
     * er, this ensures that 
     * while the shooter is not in use it is not running
     * 
     * We use closed loop control instead of run velocity because it while fight better,
     * keeps the note in place nicely
     */
    //Shooter.setDefaultCommand(Shooter.shooterDefaultCommand(0));
    Shooter.setDefaultCommand(Shooter.runVelocity(()-> 0));

    /*
     * Default command for the intake, this ensures that
     * while the intake is not in use it is not running
     */
    Intake.setDefaultCommand(Intake.runIntakeAndIndexerPercent(0.0));

    /*
     * Default command for the climber, 
     * it reads the y-axis from each controller and sets each climber to speed
     */
    Climber.setDefaultCommand
    (
      Climber.runClimberMotors(climberLeft, climberRight)
    );
  }


  /*
   * If we are already amplified, de amplify
   * If we are not amplified, amplify
   */
  private Command amplifyButtonCommand(){
    return new ConditionalCommand(
      
      // Command on true
      LEDs.deAmplify(), 

      // Command on false
      LEDs.amplifyLED(), 

      // Boolean supplier condition
      ()-> LEDs.getColor().equals("amplify"));
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
    .andThen
    (
    
    // (runs the shooter to 0 velocity and run intake and indexer) until we get a beambreak
    race(

        // LED is for driver preference, so he knows when the robot is still intaking
        LEDs.setColorToGreen()
        .andThen
        (
          Intake.runIntakeAndIndexerPercent(0.5)
          .until
          (
            Intake::getBeamBreak
          )
        ),

        Shooter.runVelocity(()-> 0)
      )
    )

    // LED to orange means driver can drive away, won't effect the intake, we have the note
    .andThen
    (
      LEDs.setColorToOrange()
    )

    // runs the indexer backwards using closed loop control,
    // we do this because the shooter wheels need room to spin up to speed
    .andThen
    (
      Intake.indexerClosedLoopControl(0.2, 0.25)
      .until
      (
        Intake::getBeamMade
      )
    )
    .andThen
    (
      Intake.setNoteStatus(true)
    )

    // moves the arm to shoot position, 
    // we default to shoot position until told otherwise by driver
    .andThen
    (
      armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS)
    )

    .andThen
    (
      Shooter.setScoringStatus("speaker")
    )

    // the withName is for names commands
    .withName("Intake Note")
    ;
  }

  private Command spinShooterToSpeed(int RPM){
    return Shooter.runVelocity(()-> ((double) RPM/Constants.Shooter.RPM_MAX));
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
  private Command shoot(int RPM) {
    return race(
      
      // run the shooter wheels to speed
      Shooter.runVelocity(()-> ((double) RPM/Constants.Shooter.RPM_MAX)),

      // wait for shooters to get up to speed
      waitUntil(Shooter::isRunning)
        .andThen(waitUntil(Shooter::shooterIsUpToSpeed))
        .andThen(

            race(

              // the shooters are now up to speed, so we run the indexers forward full speed
              Intake.runIndexerToSpeed(1),

              // the following logic watches for a velocity dip in the shooter wheels
              waitUntil(Shooter::shooterIsNotUpToSpeed)
                .andThen(new WaitCommand(0.1))
                .andThen(
                  waitUntil(Shooter::shooterIsUpToSpeed)
                      )
                )
        )
            
    )

    // tells us that we do not have a note in possesion anymore
    .andThen(Intake.setNoteStatus(false))

    // turns the led's off, this tells the driver we do not have a note anymore
    .andThen(LEDs.turnColorOff())
    ;
  }


  // /*
  //  * Sausages the note in the shooter wheels, gets us ready for amp scoring
  //  */
  // private Command sausageNote(){
  //     return 
  //         // Uses closed loop control in indexer to move the note into the shooter wheel
  //         Intake.indexerClosedLoopControl(1.5, 1)

  //         // Moves the note into place by using closed loop control
  //         .andThen(
  //           race(
  //             // ends once it reaches the desired amount of rotations (0.75 in this case)
  //             Shooter.closedLoopRotation(0.75, 1, 0.0),

  //             // Doesn't end
  //             Intake.runIndexerToSpeed(0.2)
  //           )
  //         );
  // }
  

  // /*
  //  * Gives a quick closed loop rotation to give the note a quick push into the amp
  //  */
  // private Command flickToAmp(){

  //   // kp will most likely be subject to tuning
  //   //0.17
  //   return Shooter.closedLoopRotation(1.1, 0.12, 0.0);
  // }


  // private Command sausageToBeamBreak(){
  //   return Shooter.runVelocity(()-> 0.025).until(()-> Intake.getBeamBreak())
  //           .andThen(Shooter.closedLoopRotation(0, 0, 0));
  // }


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
      Arm::resetStateToPresent,

      // ** EXECUTE **
      ()-> Arm.setGoal(position),

      // ** ON INTERRUPTED **
      interrupted->{},

      // ** END CONDITION **
      Arm::atGoal,

      // ** REQUIREMENTS **
      Arm)

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
          .andThen(
            parallel
            (
              AmpBar.closedLoopControl(12),
              shoot(1000)
            ).withTimeout(1)
          )
          .andThen(new WaitCommand(0.4))
          .andThen(AmpBar.closedLoopControl(0))
          .andThen(LEDs.turnColorOff())
          ),

          // // Flicks the note into the amp
          // .andThen(sausageToBeamBreak())
          // .andThen(
          // flickToAmp()

          // // The wait command is here because if we move the arm back to intake position 
          // //immediatly after amp scoring, if we miss the note will bounce back and get stuck in the robot
          // .andThen(LEDs.turnColorOff())
          // .andThen(new WaitCommand(1.0)))),

          
        Map.entry("speaker", 

          // Moves the arm into position for shooting
          armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS)

          // Shoots the note into speaker
          .andThen(shoot(Constants.Shooter.RPM_SPEAKER)).withTimeout(2)
          .andThen(LEDs.turnColorOff())),


        Map.entry("stockpile", 

          // Moves arm into position for stockpiling
          armToPosition(Constants.Arm.STOCKPILE_POSITION_RADIANS)

          // Shoots the note at full speed
          .andThen(shoot(Constants.Shooter.RPM_SPEAKER)).withTimeout(2))
          
      ),

    // condition (boolean supplier)
    ()-> Shooter.getScoringStatus());
  }


  public Command scorePreload(){
    return armToPosition(Constants.Arm.SHOOT_POSITION_RADIANS).andThen(shoot(Constants.Shooter.RPM_SPEAKER));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command podiumShotCommand() {
    return Swerve.driveToPose(Constants.Feild.Red.PODIUM_SCORING_LOCATION.getPose());
  }






}
