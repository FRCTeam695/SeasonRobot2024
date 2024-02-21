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
    
    CANSparkMax intakeMotor;

    CANSparkMax indexMotor;

    CANSparkFlex shootMotor1;
    CANSparkFlex shootMotor2;

    DigitalInput beamBreak;
    boolean noteStatus;

    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);

        indexMotor = new CANSparkMax(Constants.Intake.INDEX_MOTOR_ID, MotorType.kBrushless);

        indexMotor.setIdleMode(IdleMode.kBrake);

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
        intakeMotor.set(-speed);
    }

    public void runIndexerToSpeed(double speed){
        indexMotor.set(-speed);
    }
}
