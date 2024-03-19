package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import java.lang.Thread;

public class IntakeSubsystem extends SubsystemBase{
    
    private CANSparkMax intakeMotor;

    private CANSparkMax indexMotor;
    private SparkPIDController indexerPIDController;
    private RelativeEncoder indexerEncoder;

    DigitalInput beamBreak;
    boolean noteStatus;

    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.clearFaults();
    
        indexMotor = new CANSparkMax(Constants.Intake.INDEX_MOTOR_ID, MotorType.kBrushless);
        indexMotor.restoreFactoryDefaults();
        indexMotor.clearFaults();
        indexerEncoder = indexMotor.getEncoder();
        indexerPIDController = indexMotor.getPIDController();

        indexMotor.setIdleMode(IdleMode.kBrake);

        beamBreak = new DigitalInput(Constants.Intake.BEAMBREAK_ID);
        noteStatus = false;
        //runIntakeAndIndexerPercent(0);
    }

    public Command setNoteStatus(boolean hasNote){
        return runOnce(()-> noteStatus = hasNote);
    }

    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public boolean getBeamMade(){
        return !getBeamBreak();
    }

    public boolean getNoteStatus(){
        return noteStatus;
    }

    public Command runIntakeAndIndexerPercent(double percentVBus){
        return run(
            ()-> {
                runIntakeToSpeed(percentVBus);
                indexMotor.set(-percentVBus);
            }
        );
    }

    public void runIntakeToSpeed(double speed){
        intakeMotor.set(-speed);
    }

    public Command runIndexerToSpeed(double speed){
        return run(
            ()-> indexMotor.set(-speed)
        );
    }

    public Command indexerClosedLoopControl(double rotations, double kp){
        return new FunctionalCommand(
            ()-> {
                  indexerEncoder.setPosition(0);
                  indexerPIDController.setP(kp);
                 },
            ()-> indexerPIDController.setReference(-rotations, CANSparkMax.ControlType.kPosition),
            interrupted-> {},
            ()-> indexerEncoder.getPosition() <= -rotations,
            this
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beambreak", getBeamBreak());
    }
}
