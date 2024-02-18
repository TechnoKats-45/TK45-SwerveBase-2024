package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer 
{
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    private final Shoulder s_Shoulder = new Shoulder();
    private final Intake s_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();
    private final Feeder s_Feeder = new Feeder();
    private final Limelight s_Limelight = new Limelight();

    private final SendableChooser<Command> chooser; 

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls */  // For Swerve
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Variables */
    boolean driveStatus = false;
    double setPoint;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
      // TODO - add registered commands here
      /*
      NamedCommands.registerCommand("AutoFeed", new AutoFeed(s_Intake, s_Shoulder, s_Feeder));
      NamedCommands.registerCommand("AutoIntake", new AutoIntake(s_Intake, s_Feeder));
      */

      s_Swerve.setDefaultCommand
      (
        new TeleopSwerve
        (
          s_Swerve, 
          driver
        )
      );

      
      s_Intake.setDefaultCommand
      (
        new InstantCommand(() -> s_Intake.holdTarget())
      );

      s_Feeder.setDefaultCommand
      (
        new InstantCommand(() -> s_Feeder.holdTarget())
      );
      
      s_Shoulder.setDefaultCommand
      (
        new InstantCommand(() -> s_Shoulder.holdTarget())
      );
      
      s_Shooter.setDefaultCommand
      (
        new InstantCommand(() -> s_Shooter.holdTarget())
      );

      /*
      limelight.setDefaultCommand
      (
        new TeleopLimelight(limelight)
      );
      */
      
      // Configure the button bindings
      configureButtonBindings();
      s_Swerve.gyro.setYaw(0);

      chooser = AutoBuilder.buildAutoChooser("Test");
      SmartDashboard.putData("Auto Choices", chooser);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    private void configureButtonBindings()
    {
      // Driver Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////

      // RT - Automatic Fire
      driver.rightTrigger().whileTrue(Commands.sequence
      (
        
      ));      // TODO - write a function to light green LEDs if alligned and ready to be shot
    
      // LT - Automatic Aim (X and Y)
      driver.leftTrigger().whileTrue(Commands.parallel
      (
        new TeleopLimelightTurret // Auto Aim X - Swerve
        (
          s_Limelight,
          s_Shoulder,
          s_Swerve,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis)
        ),
        //new AutoAimY(s_Shoulder, s_Limelight),   // Auto Aim Y - Shoulder // TODO - MAKE THIS NOT PARALLEL
        new InstantCommand(() -> s_Shooter.setTarget(Constants.Shooter.shooterSpeed))
      ));

      // RB - Automatic Intake / Set Feed Angle / Feed
      driver.rightBumper().onTrue(Commands.sequence
      (
        new AutoIntake(s_Intake, s_Feeder),
        new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.handoffAngle)),
        new AutoFeed(s_Intake, s_Shoulder, s_Feeder)
      ));

      driver.b().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
      




      
      // Operator Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Y - Speaker Preset
      operator.y().onTrue(new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.speakerScoreAngle))); // Move to speaker preset angle (when against sub wall)

      // A - Amp Preset
      operator.a().onTrue(new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.ampScoreAngle)));         // Move to amp preset angle (when against amp wall)

      // RT - Speaker and Amp Shoot (Depending on angle) - X FOR NOW FOR TESTING!
      operator.rightTrigger().whileTrue(Commands.parallel
      (
        new InstantCommand(() -> s_Shooter.setTarget(Constants.Shooter.shooterSpeed))
        //new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.speakerScoreAngle))
      ));

      // LB - Manual Intake
      operator.leftBumper().whileTrue(new InstantCommand(() -> s_Intake.setTarget(Constants.Intake.intakeSpeed)));
    }
    
    public void printValues()
    {
        SmartDashboard.putNumber("Swerve Yaw", s_Swerve.gyro.getYaw());
        SmartDashboard.putNumber("Swerve Pitch", s_Swerve.gyro.getPitch()); 
        SmartDashboard.putNumber("Swerve Roll", s_Swerve.gyro.getRoll());
        
        SmartDashboard.putBoolean("Intake GamePiece Detected", s_Intake.detectGamePiece());
        SmartDashboard.putBoolean("Feeder GamePiece Detected", s_Feeder.detectGamePiece());
        SmartDashboard.putNumber("Shoulder Angle", s_Shoulder.getAngle());

        SmartDashboard.putNumber("Limelight Updates", s_Limelight.getUpdates());
        SmartDashboard.putBoolean("Target Detected", s_Limelight.tagExists());
        SmartDashboard.putNumber("LimeLight X", s_Limelight.getRX());
        SmartDashboard.putNumber("LimeLight Y", s_Limelight.getRY());
        SmartDashboard.putNumber("LimeLight Z", s_Limelight.getRZ());
        SmartDashboard.putNumber("LimeLight Lateral Offset", s_Limelight.getLateralOffset());
        // TODO
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
      return chooser.getSelected();
    }
}