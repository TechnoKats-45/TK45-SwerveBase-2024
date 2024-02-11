package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class TeleopLimelightTurret extends Command 
{
    private final Limelight limelight;
    private final Swerve swerve;
    private final DoubleSupplier translation;
    private final DoubleSupplier strafe;
    private final BooleanSupplier robotCentric;

    public TeleopLimelightTurret(Limelight limelight, Swerve swerve, DoubleSupplier translation, DoubleSupplier strafeSup, BooleanSupplier robotCentric) 
    {
        this.limelight = limelight;
        this.swerve = swerve;
        this.translation = translation;
        this.strafe = strafeSup;
        this.robotCentric = robotCentric;
        addRequirements(this.limelight, swerve);
    }

    public TeleopLimelightTurret(Limelight limelight, Swerve swerve) 
    {
        this.limelight = limelight;
        this.swerve = swerve;
        this.translation = () -> 0.0;
        this.strafe = () -> 0.0;
        this.robotCentric = () -> false;
        addRequirements(this.limelight, swerve);
    }

    /*
     * how to go to apriltag:
     * find tag (duh)
     * find position of tag relative to robot
     * rotate from current position to tag using rotation PID controller
     * variate PID magnitude by distance factor
     * allow turret mode ("locked" rotation to tag)
     */
    @Override
    public void execute() 
    {
        limelight.refreshValues();

        /* Apply Deadband */
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.STICK_DEADBAND);

        /* Calculate Rotation Magnitude */
        try 
        (
            PIDController rotController = new PIDController
            (
                Constants.Limelight.LIMELIGHT_P,
                Constants.Limelight.LIMELIGHT_I,
                Constants.Limelight.LIMELIGHT_D
            )
        ) 
        {
            rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

            double rotate = rotController.calculate
            (
                swerve.getYaw(),
                swerve.getYaw() + (limelight.getX() < 0 ? -1 : 1) * Math.atan(Math.pow(limelight.getX(), 2) / Math.pow(limelight.getZ(), 2))
                // TODO - Check above formula
            );

            /* Drive */
            swerve.drive
            (
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                -rotate,
                !robotCentric.getAsBoolean(),
                true
            );
        }
    }
}