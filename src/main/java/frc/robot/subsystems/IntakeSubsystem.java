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

    private void runSubsystemToSpeed(double speed){
        runIntakeToSpeed(speed);
        runIndexerToSpeed(speed);
    }

    private void runIntakeToSpeed(double speed){
        intakeMotor1.set(-speed);
        intakeMotor2.set(-speed);
    }

    private void runIndexerToSpeed(double speed){
        indexMotor1.set(speed);
        indexMotor2.set(speed);
    }

    @Override
    public void periodic(){
        if(stage == 1){  // stage 1 is jamming the note into the shooter motors until the beambreak is broken
            if(beamBreak.get()){ // if beambreak is broken
                speed = 0;
                stage = 2;
                runSubsystemToSpeed(0);
            }
            else{
                runSubsystemToSpeed(speed);
            }
        }
        else if(stage == 2){  // stage 2 is rocking back the indexer until the beam is unbroken
            if(!beamBreak.get()){ // if beambreak is unbroken
                stage = 1;
                runIndexerToSpeed(0);
            }
            else{
                runIndexerToSpeed(-0.1);
            }
        }
    }


}
