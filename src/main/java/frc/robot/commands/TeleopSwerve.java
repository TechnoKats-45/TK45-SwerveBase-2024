package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private double translationSup;
    private double strafeSup;
    private double rotationSup;
    private CommandXboxController controller;
    private boolean robotCentric;

    public TeleopSwerve(Swerve s_Swerve, CommandXboxController controller) 
    {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.controller = controller;
    }

    @Override
    public void execute() 
    {
        // Adjust speed using the custom function:
        translationSup = adjustSpeed(-controller.getLeftY());
        strafeSup = adjustSpeed(-controller.getLeftX());

        rotationSup = -controller.getRightX() * 0.8;    // Was 0.7
        robotCentric = !controller.x().getAsBoolean();

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup, Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup, Constants.STICK_DEADBAND);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            robotCentric,
            true
        );
    }

    // Here's the adjustSpeed method integrated into your TeleopSwerve class
    private double adjustSpeed(double input) 
    {
        final double slowSpeedMultiplier = 0.5; // Adjust for desired slow speed
        final double threshold = 0.5; // The input value at which the speed adjustment strategy changes
        final double maxSpeed = 1.0; // The maximum output speed

        if (Math.abs(input) < threshold) 
        {
            return input * slowSpeedMultiplier;
        }
        else 
        {
            // Calculate the linear growth rate for the fast section
            double linearGrowthRate = (maxSpeed - threshold * slowSpeedMultiplier) / (1 - threshold);
            // Apply the linear growth rate beyond the threshold
            return Math.signum(input) * (linearGrowthRate * (Math.abs(input) - threshold) + threshold * slowSpeedMultiplier);
        }
    }
}
