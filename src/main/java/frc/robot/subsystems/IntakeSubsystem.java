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
    int stage;


    public IntakeSubsystem(){
        intakeMotor1 = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_1_PORT, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_2_PORT, MotorType.kBrushless);

        indexMotor1 = new CANSparkMax(Constants.Intake.INDEX_MOTOR_1_PORT, MotorType.kBrushless);
        indexMotor2 = new CANSparkMax(Constants.Intake.INDEX_MOTOR_2_PORT, MotorType.kBrushless);

        beamBreak = new DigitalInput(0);

        runSubsystemToSpeed(0);
        speed = 0;
        stage = 0;
    }

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
        if(stage == 0){
            if(beamBreak.get()){
                speed = 0;
                stage = 1;
                runSubsystemToSpeed(0);
            }
            else{
                runSubsystemToSpeed(speed);
            }
        }
        else if(stage == 1){
            if(!beamBreak.get()){
                stage = 0;
                speed = 0;
                runSubsystemToSpeed(0);
            }
            else{
                runSubsystemToSpeed(-0.1);
            }
        }
    }


}
