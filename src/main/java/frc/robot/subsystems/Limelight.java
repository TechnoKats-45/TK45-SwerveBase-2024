package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase 
{
    private final DoubleArraySubscriber tagPoseTopic;
    private int updates;
    private double[] tagPose;

    
    public Limelight() 
    {
        tagPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        tagPoseTopic = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
        tagPose = new double[6];
    }

    @Override
    public void periodic() 
    {
        refreshValues();
    }

    public int getUpdates() 
    {
        return updates;
    }

    // This works! 2-10-24 JTL
    public double getTV() // Whether the limelight has any valid targets (0 or 1)
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    }

    
    // X+ is to the right if you are looking at the tag
    public double getX() 
    {
        refreshValues();
        return tagPose[0];
    }

    // Y+ is upwards
    public double getY()
    {
        refreshValues();
        return tagPose[1];   
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    public double getZ() 
    {
        refreshValues();
        return tagPose[2];
    }

    public double getRoll() 
    {
        refreshValues();
        return tagPose[3];
    }
    
    public double getPitch()
    {
        refreshValues();
        return tagPose[4];
    }

    public double getYaw() 
    {
        refreshValues();
        return tagPose[5];
    }

    public double getLatency()
    {
        refreshValues();
        return tagPose[6];
    }

    public void refreshValues()
    {        
        tagPose = tagPoseTopic.get(new double[6]);
        tagPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        updates++;
    }
}