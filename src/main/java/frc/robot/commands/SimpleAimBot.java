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
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shooter;

public class SimpleAimBot extends Command
{
    private final Limelight s_Limelight;
    private final Shoulder s_Shoulder;
    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final CommandXboxController controller;
    private final Joystick rumbleController;
    private double translationSup;
    private double strafeSup;
    private double rotationSup;
    private boolean robotCentric;
    private PIDController rotController;

    public SimpleAimBot
    (
        Limelight s_Limelight,
        Shoulder s_Shoulder,
        Swerve s_Swerve,
        CommandXboxController controller,
        Joystick rumbleController,
        Shooter s_Shooter
    )
    {
        this.s_Limelight = s_Limelight;
        this.s_Shoulder = s_Shoulder;
        this.s_Swerve = s_Swerve;
        this.controller = controller;
        this.rumbleController = rumbleController;
        this.s_Shooter = s_Shooter;
        addRequirements(s_Limelight, s_Shoulder, s_Swerve, s_Shooter);

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
        s_Shooter.setTarget(Constants.Shooter.shooterSpeed);
        s_Shooter.holdTarget();

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
                NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
                NetworkTableEntry ty = table.getEntry("ty");
                double targetOffsetAngle_Vertical = ty.getDouble(0.0);

                // how many degrees back is your limelight rotated from perfectly vertical?
                double limelightMountAngleDegrees = 35.0;

                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

                if(targetOffsetAngle_Vertical == 0)    // If angle is erroneous (Limelight missing / not working)
                {
                    // Set to preset angle
                    angleToGoalDegrees = 35;    // TODO - change to actual value
                }

                double shoulderAngleToGoalDegrees = Constants.Shoulder.groundParallelAngle - angleToGoalDegrees;  // groundParallelAngle is the angle of the shoulder when it is parallel to the ground

                s_Shoulder.setTarget(shoulderAngleToGoalDegrees);
                s_Shoulder.holdTarget();
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

        // Indicator LEDs on Limelight and Vibration
        if(s_Limelight.tagExists() && s_Swerve.isRotAligned() && s_Shoulder.isAligned() && s_Shooter.upToSpeed())    // If both aligned and up to speed
        {
            s_Limelight.setLEDMode(Constants.Limelight.LED_ON);
            // Controller vibrate ON
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 1);
        }
        else if(s_Limelight.tagExists() && s_Swerve.isRotAligned() && s_Shoulder.isAligned())   // If both are aligned, but not up to speed - NO vibrate and solid LED
        {
            s_Limelight.setLEDMode(Constants.Limelight.LED_ON);
            // Controller vibrate OFF
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);
        }
        else if(!s_Swerve.isRotAligned() || !s_Shoulder.isAligned())    // If 1/2 is aligned - off / on vibrate and blink LED
        {
            s_Limelight.setLEDMode(Constants.Limelight.LED_BLINK); 
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);
        }
        else if(!s_Swerve.isRotAligned() && !s_Shoulder.isAligned())    // If 0/2 are aligned - no vibrate, LEDs off
        {
            s_Limelight.setLEDMode(Constants.Limelight.LED_OFF);
            // controller vibrate OFF
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);
        }
        else
        {
            // I should not be here - ERROR
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        SmartDashboard.putBoolean("AIMED", false);
        s_Shoulder.setTarget(s_Shoulder.getAngle());
        s_Shoulder.holdTarget();
        rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);
        s_Shooter.coastToZero();
    }

    @Override 
    public boolean isFinished()
    {
        if (s_Shoulder.isAligned() && s_Swerve.isRotAligned() && RobotState.isAutonomous())
        {
            return true;  // JTL 3-16-24 Changed to attempt to fix auto
            //return false;
        }
        else
        {
            return false;
        }
    }
}
