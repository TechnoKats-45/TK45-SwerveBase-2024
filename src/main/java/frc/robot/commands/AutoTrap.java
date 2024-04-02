package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.SecondLimelight;
import frc.robot.subsystems.Swerve;

import frc.robot.Constants;

public class AutoTrap extends Command 
{
    private Shooter s_Shooter;
    private Shoulder s_Shoulder;
    private Feeder s_Feeder;
    private SecondLimelight s_SecondLimelight;
    private Swerve s_Swerve;
    private double rotationTarget;

    public AutoTrap(Shooter s_Shooter, Shoulder s_Shoulder, Feeder s_Feeder, SecondLimelight s_SecondLimelight, Swerve s_Swerve) 
    {
        this.s_Shooter = s_Shooter;
        this.s_Shoulder = s_Shoulder;
        this.s_Feeder = s_Feeder;
        this.s_SecondLimelight = s_SecondLimelight;
        this.s_Swerve = s_Swerve;

        addRequirements(s_Shooter, s_Shoulder, s_Feeder, s_SecondLimelight, s_Swerve);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()   //     public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) 
    {        
        // Calculate bot rotation target to arpil tag to maintain parallelism
            rotationTarget = calculateParallelAngle();   // calculate parallel angle

        // Calculate bot distance from april tag
            calculateDistanceToAprilTag();

        // Calculate bot left / right alignment distance from april tag.
            //calculateTranslation();
            if(s_Swerve.isRotAligned())
            {

            }

        // Drive to correct location (translation (x,y) and rotation)

        // Aim shoulder at right angle

        // Spool up shooter

        // Activate leaf blower
            // Use Spike Relay (controlled over PWM) to activate leaf blower

        // Activate feeder to shoot note into trap
            if(s_Shoulder.isAligned() && s_Swerve.isRotAligned() && s_Swerve.isTranslationAligned() && s_Shooter.upToSpeed())
            {

            }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    public double calculateParallelAngle()
    {
        // Get the yaw value of the AprilTag as perceived by the Limelight
        double aprilTagYaw = s_SecondLimelight.getYaw();

        double robotCurrentYaw = s_Swerve.getYaw();

        // Calculate the desired orientation to maintain parallelism.
        double desiredParallelAngle = robotCurrentYaw - aprilTagYaw;

        // Normalize the desired angle to the range [-180, 180] for consistency
        desiredParallelAngle = Rotation2d.fromDegrees(desiredParallelAngle).getDegrees();

        return desiredParallelAngle;
    }

    public double calculateDistanceToAprilTag()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SecondLimelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 60.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    /*
    public double calculateTranslation()
    {
        if(s_Swerve.isRotAligned())
        {
            
        }
    }
    */
}