// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/*
 * TARGET TRACKING is best when high and level-
 */

public class VisionSubsystemLimelight extends SubsystemBase {
  /** Creates a new VisionSubsystemLimelight. */

  Field2d m_field;
  LimelightHelpers.LimelightResults llresults;

  public VisionSubsystemLimelight() {
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
    // LimelightHelpers.setLEDMode_ForceOff("");
  }

  @Override
  public void periodic() {
  }

  public Pose2d getBotPose2d() {
    try {
      Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue("");
      m_field.setRobotPose(pose);
      SmartDashboard.putString("robotPose", pose.toString());
    } catch (Exception e) {
      SmartDashboard.putString("robotPose", "none");
      m_field.setRobotPose(new Pose2d());
    }
    SmartDashboard.putString("BotPose2d", LimelightHelpers.getBotPose2d_wpiBlue("").toString());

    return LimelightHelpers.getBotPose2d_wpiBlue("");
  }

  public double getCameratoTargetDistance() {
    double cameraToTargetRadians = (Constants.VisionLimelight.CAMERA_MOUNT_DEGREES +
        LimelightHelpers.getTY("")) * 3.14159 / 180;

    double distanceFromLimelightToTargetMeters = (Constants.VisionLimelight.TARGET_HEIGHT_METERS
        - Constants.VisionLimelight.CAMERA_HEIGHT_METERS)
        / Math.tan(cameraToTargetRadians);
    return distanceFromLimelightToTargetMeters;

  }

  public Pose3d getBotPose3d() {
    // try {
    // Pose2d pose = LimelightHelpers.getBotPose2d("");
    // m_field.setRobotPose(pose);
    // SmartDashboard.putString("robotPose", pose.toString());
    // } catch (Exception e) {
    // SmartDashboard.putString("robotPose", "none");
    // m_field.setRobotPose(new Pose2d());
    // }
    SmartDashboard.putString("BotPose3d", LimelightHelpers.getBotPose3d_wpiBlue("").toString());
    return LimelightHelpers.getBotPose3d_wpiBlue("");
  }
}
