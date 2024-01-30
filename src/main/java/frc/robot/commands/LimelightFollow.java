package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;


public class LimelightFollow extends Command 
{
    private Swerve s_Swerve;
    private Limelight s_limelight;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    /** Creates a new TeleopFeeder. */
    public LimelightFollow(Limelight limelight, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) 
    {
        s_limelight = limelight;
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(s_limelight, s_Swerve);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_limelight.refreshValues();

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double distFactor = 0.2 * (s_limelight.getRZ() > 5 ? 5 : s_limelight.getRZ());

        /* Get rotation */
        PIDController rotController = new PIDController((1.0-(0.75*distFactor))*0.2, 0.0001, 0.000005);
        rotController.enableContinuousInput(-180, 180);
        double rotate = rotController.calculate(s_Swerve.gyro.getYaw().getValue(), s_Swerve.getYaw().getDegrees() + 15*s_limelight.getRX());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            -rotate,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}