package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import java.lang.Thread;

public class IntakeSubsystem extends SubsystemBase{
    
    CANSparkMax intakeMotor1;
    CANSparkMax intakeMotor2;

    CANSparkMax indexMotor1;
    CANSparkMax indexMotor2;

    CANSparkFlex shootMotor1;
    CANSparkFlex shootMotor2;

    DigitalInput beamBreak;
    boolean noteStatus;

    public IntakeSubsystem(){
        intakeMotor1 = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_1_PORT, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_2_PORT, MotorType.kBrushless);

        indexMotor1 = new CANSparkMax(Constants.Intake.INDEX_MOTOR_1_PORT, MotorType.kBrushless);
        indexMotor2 = new CANSparkMax(Constants.Intake.INDEX_MOTOR_2_PORT, MotorType.kBrushless);

        indexMotor1.setIdleMode(IdleMode.kBrake);
        indexMotor2.setIdleMode(IdleMode.kBrake);

        beamBreak = new DigitalInput(0);
        noteStatus = false;
        runIntakeAndIndexerPercent(0);
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
        intakeMotor1.set(-speed);
        intakeMotor2.set(-speed);
    }

    public void runIndexerToSpeed(double speed){
        indexMotor1.set(speed);
        indexMotor2.set(speed);
    }
}
