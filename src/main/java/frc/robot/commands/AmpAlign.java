package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
        // TODO - switch pipeline
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
            // Caculate April Tag / Bot Parallel Angle
            double aprilTagYaw = s_Limelight.getYaw();  // Get the yaw value of the AprilTag as perceived by the Limelight
            double robotCurrentYaw = s_Swerve.getYaw();
            double desiredParallelAngle = robotCurrentYaw - aprilTagYaw;    // Calculate the desired orientation to maintain parallelism.
            desiredParallelAngle = Rotation2d.fromDegrees(desiredParallelAngle).getDegrees();   // Normalize the desired angle to the range [-180, 180] for consistency

            // TODO - Do stuff
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