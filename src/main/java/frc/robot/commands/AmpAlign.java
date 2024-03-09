package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class AmpAlign extends Command 
{
    private Feeder s_Feeder;
    private Shoulder s_Shoulder;
    private Limelight s_Limelight;
    private Swerve s_Swerve;
    private CommandXboxController controller;

    private double translationSup;
    private double strafeSup;
    private double rotationSup;
    private boolean robotCentric;
    private PIDController rotController;
    private PIDController strafeController;
    private boolean finished = false;

    /** Creates a new TeleopFeeder. */
    public AmpAlign(Feeder s_Feeder, Shoulder s_Shoulder, Limelight s_Limelight, Swerve s_Swerve, CommandXboxController controller) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Shoulder = s_Shoulder;
        this.s_Limelight = s_Limelight;
        this.s_Swerve = s_Swerve;
        this.controller = controller;
        
        addRequirements(s_Feeder, s_Shoulder, s_Limelight, s_Swerve);

        rotController = new PIDController
        (
            Constants.Limelight.LIMELIGHT_P,
            Constants.Limelight.LIMELIGHT_I,
            Constants.Limelight.LIMELIGHT_D
        );

        strafeController = new PIDController
        (
            Constants.Limelight.LIMELIGHT_P,
            Constants.Limelight.LIMELIGHT_I,
            Constants.Limelight.LIMELIGHT_D
        );

        rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

        s_Shoulder.setTarget(Constants.Shoulder.handoffAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
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
        double rotate = MathUtil.applyDeadband(rotationSup, Constants.STICK_DEADBAND);

        if(s_Limelight.tagExists()) // If April Tag exists
        {
            s_Shoulder.holdTarget();

            // Calculate the rotation value for swerve
                rotate = -rotController.calculate
                (
                    s_Swerve.getYaw(),
                    s_Swerve.getYaw() + s_Limelight.getLateralOffset()
                );

            // Strafe target Computation
                double tx = s_Limelight.getRX();  // Get horizontal offset from the target
                double targetTx = 0;    // The goal is to have the target centered, hence targetTx is 0

            // Calculate the strafe value for swerve
                strafeVal = -strafeController.calculate
                (
                    tx,
                    targetTx
                );

            /* Drive */
                s_Swerve.drive
                (
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                    rotate,
                    robotCentric,
                    true
                );

                if(s_Swerve.isRotAligned() && s_Limelight.isAlignedX())    // Check if swerve is aligned with the target
                {
                    s_Shoulder.setTarget(Constants.Shoulder.ampScoreAngle);
                    s_Shoulder.holdTarget();

                    if(s_Shoulder.isAligned())
                    {
                        s_Feeder.runFeeder(Constants.Feeder.ampScoreSpeed);
                        Timer.delay(0.25);
                        s_Feeder.runFeeder(0);
                        finished = true;
                    }
                }
                else
                {
                    s_Shoulder.holdTarget();
                }
        }
        else
        {
            /* Drive Normally */
            s_Swerve.drive
            (
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotate,
                robotCentric,
                true
            );
        }
    }

    @Override
    public boolean isFinished()
    {
        if(finished == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Feeder.runFeeder(0);
        s_Shoulder.holdTarget();
        // Change shoulder to stow position? Handoff angle
    }
}