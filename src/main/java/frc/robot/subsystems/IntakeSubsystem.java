package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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

    CANSparkFlex shootMotor1;
    CANSparkFlex shootMotor2;

    DigitalInput beamBreak;
    boolean noteStatus;

    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);

        indexMotor = new CANSparkMax(Constants.Intake.INDEX_MOTOR_ID, MotorType.kBrushless);
        indexerEncoder = indexMotor.getEncoder();
        indexerPIDController = indexMotor.getPIDController();

        indexMotor.setIdleMode(IdleMode.kBrake);

        beamBreak = new DigitalInput(2);
        noteStatus = false;
        //runIntakeAndIndexerPercent(0);
    }

    public void setNoteStatus(boolean hasNote){
        noteStatus = hasNote;
    }

    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public boolean getNoteStatus(){
        return noteStatus;
    }

    public void runIntakeAndIndexerPercent(double percentVBus){
        runIntakeToSpeed(percentVBus);
        runIndexerToSpeed(percentVBus);
    }

    public void runIntakeToSpeed(double speed){
        intakeMotor.set(-speed);
    }

    public void runIndexerToSpeed(double speed){
        indexMotor.set(-speed);
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
