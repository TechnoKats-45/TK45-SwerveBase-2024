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
import frc.robot.subsystems.Shoulder;

public class TeleopLimelightTurret extends Command 
{
    private final Limelight s_Limelight;
    private final Shoulder s_Shoulder;
    private final Swerve s_Swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;

    public TeleopLimelightTurret
    (
        Limelight s_Limelight,
        Shoulder s_Shoulder,
        Swerve s_Swerve,
        DoubleSupplier translationSup,
        DoubleSupplier strafeSup
    ) 
    {
        this.s_Limelight = s_Limelight;
        this.s_Shoulder = s_Shoulder;
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        addRequirements(s_Swerve);
    }
    
    @Override
    public void execute() 
    {
        /* Apply Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Calculate Rotation Magnitude */
        if(s_Limelight.tagExists()) 
        {
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
                    true
                );
            }
        }
    }
}