// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NTTest extends SubsystemBase {
  StringSubscriber scoreLocationSub;
  BooleanSubscriber amplifySub;
  NetworkTable table;
  NetworkTableInstance inst;

  public NTTest() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SideCar");
    
    scoreLocationSub = table.getStringTopic("Score Location").subscribe("");
    amplifySub = table.getBooleanTopic("Amplify").subscribe(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("ScoreLocation", scoreLocationSub.get());
    SmartDashboard.putBoolean("isAmplify", amplifySub.get());
  }
}
