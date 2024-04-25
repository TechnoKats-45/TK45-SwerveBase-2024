package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase 
{
    private final DoubleArraySubscriber tagPoseTopic;
    private NetworkTable table;
    private double[] tagPose;
    private int updates;

    public Limelight()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPoseTopic = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
        tagPose = new double[6];
    }

    @Override
    public void periodic() 
    {
        refreshValues();
    }

    public boolean tagExists() 
    {
        refreshValues();
        return !(getTable().getEntry("tv").getDouble(0) == 0);
    }

    public double tagID()
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    }

    public int getUpdates() 
    {
        return updates;
    }

    public double simpleY()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");   // TK45
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        targetOffsetAngle_Vertical += 35;    // Angle Offset
        return targetOffsetAngle_Vertical;
    }

    // X+ is to the right when looking at the tag
    public double getRX()   // Unafected from limelight angle change
    {
        refreshValues();
        return tagPose[0];
    }

    // Y+ is upwards
    public double getRY()
    {
        refreshValues();
        return 1.3;
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    // side, Z- is on other side of robot)
    public double getRZ()
    {
        refreshValues();
        return tagPose[2];
    }

    public double getPitch() 
    {
        refreshValues();
        return tagPose[3];
    }

    public double getYaw() 
    {
        refreshValues();
        return tagPose[4];
    }

    public double getRoll() 
    {
        refreshValues();
        return tagPose[5];
    }

    // returns lateral angle of tag from center of limelight in degrees
    public double getLateralOffset() 
    {
        refreshValues();
        return (new Rotation2d(tagPose[2], tagPose[0]).getDegrees());
    }

    public NetworkTable getTable() 
    {
        refreshValues();
        return table;
    }

    public void refreshValues() 
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPose = tagPoseTopic.get(new double[6]);
        updates++;
    }

    public boolean isAlignedX()
    {
        return Math.abs(getRX()) < Constants.Limelight.X_Alignment_Tolerance;
    }

    public void setPipeline(int pipeline) 
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }

    // Camera Controls:
    public void setLEDMode(int mode) 
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Limelight Updates", getUpdates());
        SmartDashboard.putBoolean("Target Detected", tagExists());
        SmartDashboard.putNumber("LimeLight X", getRX());
        SmartDashboard.putNumber("LimeLight Y", getRY());
        //SmartDashboard.putNumber("LimeLight Z", getRZ());
        SmartDashboard.putNumber("LimeLight Lateral Offset", getLateralOffset());

        SmartDashboard.putNumber("Simple Y", simpleY());
    }
}