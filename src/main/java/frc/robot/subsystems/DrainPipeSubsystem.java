// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrainPipeSubsystem extends SubsystemBase {

    private CANSparkMax DrainPipeMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final Timer timer = new Timer();
    // arm encoder
    private DigitalInput drainPipeDI = new DigitalInput(9);
    private DutyCycleEncoder drainPipeEncoder = new DutyCycleEncoder(drainPipeDI);
    //private PIDController drainPipePIDController = new PIDController(7.5, 0, 0);

    public DrainPipeSubsystem() {
        DrainPipeMotor.restoreFactoryDefaults();
        DrainPipeMotor.setIdleMode(IdleMode.kBrake);

    }
/*
    public int getLevel() {
        pos = armEncoder.getPosition();
        for(int i = 2; i > 0; i--){
            if (pos >= (TICK_LEVELS[i] - 1000)){
                return i;
            }
        }
        return 0;
    }
*/

    public void runDrainPipeMotor(double speed)
    {
        DrainPipeMotor.set(speed);
    }
    public void shootNote(){
        runDrainPipeMotor(1);
        timer.delay(2);
        runDrainPipeMotor(0);
    }


    @Override
    public void periodic(){

    }
}