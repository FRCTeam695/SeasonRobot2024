package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class ScoringLocation {
    
    private Pose2d pose;
    private double pitch;
    private int speed;

    public ScoringLocation(Pose2d pose, double pitch, int speed){
        this.pose = pose;
        this.pitch = pitch;
        this.speed = speed;
    }

    public Pose2d getPose(){
        return pose;
    }

    public double getPitch(){
        return pitch;
    }

    public int getSpeed(){
        return speed;
    }
}
