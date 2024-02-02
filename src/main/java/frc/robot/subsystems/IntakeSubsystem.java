package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    CANSparkMax intakeMotor1;
    CANSparkMax intakeMotor2;

    CANSparkMax indexMotor1;
    CANSparkMax indexMotor2;

    DigitalInput beamBreak;
    boolean noteStatus;

    double speed;

    /*
     * Stage keeps track of what stage of the intake we are in, there are two stages
     * 
     * 1. We are intaking at a high speed, this jams the note into our shooter, this is a problem as the
     *    shooters need room to speed up before it comes in contact with the note when its time for shooting.
     *    This calls for stage two.
     * 
     * 2. The note is currently in contact with the shooter wheels, we run the indexer backwards at 0.1 speed
     *    until the break is not broken anymore.
     */
    int stage;


    public IntakeSubsystem(){
        intakeMotor1 = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_1_PORT, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_2_PORT, MotorType.kBrushless);

        indexMotor1 = new CANSparkMax(Constants.Intake.INDEX_MOTOR_1_PORT, MotorType.kBrushless);
        indexMotor2 = new CANSparkMax(Constants.Intake.INDEX_MOTOR_2_PORT, MotorType.kBrushless);

        beamBreak = new DigitalInput(0);
        noteStatus = false;
        runSubsystemToSpeed(0);
        speed = 0;
        stage = 1;
    }

    /*
     * Depending on what stage it is the intake and/or the indexer is always being 
     * set to this speed
     */
    public void setSpeed(double speed){
        this.speed = speed;
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

    public void runSubsystemToSpeed(double speed){
        runIntakeToSpeed(speed);
        runIndexerToSpeed(speed);
    }

    public void runIntakeToSpeed(double speed){
        intakeMotor1.set(-speed);
        intakeMotor2.set(-speed);
    }

    public void runIndexerToSpeed(double speed){
        indexMotor1.set(speed);
        indexMotor2.set(speed);
    }

    @Override
    public void periodic(){
        
    }


}
