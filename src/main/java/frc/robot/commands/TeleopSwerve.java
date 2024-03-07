package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopSwerve extends Command 
{    
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
        translationSup = -controller.getLeftY();
        strafeSup = -controller.getLeftX();
        rotationSup = -controller.getRightX() * 0.7;
        robotCentric = !controller.x().getAsBoolean();

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup, Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup, Constants.STICK_DEADBAND);

        /* Drive */
        s_Swerve.drive
        (
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            robotCentric,
            true
        );
    }
}