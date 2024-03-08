package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.RobotState;


import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;

public class SimpleAimBot extends Command
{
    private final Limelight s_Limelight;
    private final Shoulder s_Shoulder;
    private final Swerve s_Swerve;
    private final CommandXboxController controller;
    private double translationSup;
    private double strafeSup;
    private double rotationSup;
    private boolean robotCentric;
    private PIDController rotController;
    private double aprilTagHeightOffset;

    public SimpleAimBot
    (
        Limelight s_Limelight,
        Shoulder s_Shoulder,
        Swerve s_Swerve,
        CommandXboxController controller,
        double aprilTagHeightOffset
    )
    {
        this.s_Limelight = s_Limelight;
        this.s_Shoulder = s_Shoulder;
        this.s_Swerve = s_Swerve;
        this.controller = controller;
        addRequirements(s_Limelight, s_Shoulder, s_Swerve);  // TODO - add

        rotController = new PIDController
        (
            Constants.Limelight.LIMELIGHT_P,
            Constants.Limelight.LIMELIGHT_I,
            Constants.Limelight.LIMELIGHT_D
        );

        rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

        s_Shoulder.setTarget(s_Shoulder.getAngle());
    }

    @Override
    public void execute()
    {
        s_Shoulder.holdTarget();

        translationSup = -controller.getLeftY();
        strafeSup = -controller.getLeftX();
        rotationSup = -controller.getRightX();
        robotCentric = !controller.x().getAsBoolean();

        /* Apply Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup, Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.STICK_DEADBAND);

        /* Calculate Rotation Magnitude */
        if(s_Limelight.tagExists())     // If the limelight sees a target, then use the limelight to aim
        {
            s_Shoulder.holdTarget();

            // Calculate the rotation value for swerve
                double rotate = -rotController.calculate
                (
                    s_Swerve.getYaw(),
                    s_Swerve.getYaw() + s_Limelight.getLateralOffset()
                );

            /* Drive */
                s_Swerve.drive
                (
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                    rotate,
                    robotCentric,
                    true
                );

            // Calculate the angle for the Shoulder
                NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");   // TK45
                NetworkTableEntry ty = table.getEntry("ty");
                double targetOffsetAngle_Vertical = ty.getDouble(0.0);

                // how many degrees back is your limelight rotated from perfectly vertical?
                double limelightMountAngleDegrees = 35.0; // TK45

                // distance from the center of the Limelight lens to the floor
                double limelightLensHeightInches = 13.93; // TK45

                // distance from the target to the floor
                double goalHeightInches = 78.13+((82.90-78.13)/2);  // Average between top and bottom of the target opening

                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                // calculate distance   // Not Needed
                double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

                double shoulderAngleToGoalDegrees = angleToGoalDegrees - Constants.Shoulder.groundParallelAngle;  // groundParallelAngle is the angle of the shoulder when it is parallel to the ground

                s_Shoulder.setTarget(shoulderAngleToGoalDegrees);

        }
        else    // If the limelight doesn't see a target, then just drive normally
        {
            s_Shoulder.holdTarget();

            // Drive Normally
            s_Swerve.drive
            (
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotationSup * Constants.Swerve.maxAngularVelocity,
                robotCentric,
                true
            );

            // Hold the shoulder in place
            s_Shoulder.holdTarget();
        }

        SmartDashboard.putString("Text", "Reached 1");

        // Indicator LEDs on Limelight
        if(s_Swerve.isRotAligned() && s_Shoulder.isAligned())  
        {
            s_Limelight.setLEDMode(Constants.Limelight.LED_ON); // Alert the driver that the robot is ready to shoot
            SmartDashboard.putBoolean("AIMED", true);

            SmartDashboard.putString("Text", "aligned");
        }
        else if(!s_Swerve.isRotAligned() || !s_Shoulder.isAligned())
        {
            s_Limelight.setLEDMode(Constants.Limelight.LED_OFF); // Alert the driver that the robot is ready to shoot
            SmartDashboard.putBoolean("AIMED", false);

            SmartDashboard.putString("Text", "not aligned");
        }
    }

    @Override
    public void end(boolean interrupted) // I have no idea if this works!
    {
        SmartDashboard.putBoolean("AIMED", false);
    }

    @Override 
    public boolean isFinished()
    {
        if (s_Shoulder.isAligned() && s_Swerve.isRotAligned() && RobotState.isAutonomous())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
