package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.util.Units;

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
    private final Limelight limelight = new Limelight();


    private final SendableChooser<Command> autoChooser;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */  // For Swerve
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kB.value);               // B BUTTON     // TODO - change binding
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);  // LEFT BUMPER  // TODO - change binding

    /* Operator Buttons */
    private final JoystickButton intakeIn = new JoystickButton(operator, XboxController.Button.kA.value);         // TODO - Update Button Config  // Left Bumper
    private final JoystickButton intakeOut = new JoystickButton(operator, XboxController.Button.kA.value);        // TODO - Update Button Config
    private final JoystickButton speakerPreset = new JoystickButton(operator, XboxController.Button.kY.value);    // TODO - Update Button Config  // Y Button
    private final JoystickButton shooterSpin = new JoystickButton(operator, XboxController.Button.kA.value);      // TODO - Update Button Config
    private final JoystickButton ampPreset = new JoystickButton(operator, XboxController.Button.kA.value);        // TODO - Update Button Config  // A Button
    private final JoystickButton feederFeed = new JoystickButton(operator, XboxController.Button.kA.value);       // TODO - Update Button Config

    /* Variables */
    boolean driveStatus = false;
    double setPoint;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
      NamedCommands.registerCommand("autoIntake", Commands.run(() -> s_Intake.autoIntake()));
      NamedCommands.registerCommand("autoAimX", Commands.run(() -> s_Swerve.autoAimX()));
      NamedCommands.registerCommand("autoAimY", Commands.run(() -> s_Shoulder.autoAimY()));   
      
      s_Swerve.setDefaultCommand
      (
        new TeleopSwerve
        (
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis), 
          () -> robotCentric.getAsBoolean()
        )
      );

      /*
      s_Intake.setDefaultCommand
      (
        new TeleopIntake
        (
          s_Intake, 
          operator,
          driver
        )      
      );

      s_Feeder.setDefaultCommand
      (
        new TeleopFeeder
        (
          s_Feeder,
          operator,
          driver
        )
      );

      s_Shoulder.setDefaultCommand
      (
        new TeleopShoulder
        (
          s_Shoulder,
          operator,
          driver
        )
      );

      s_Shooter.setDefaultCommand
      (
        new TeleopShooter
        (
          s_Shooter,
          operator,
          driver
        )
      );

      limelight.setDefaultCommand
      (
        new TeleopLimelight(limelight)
      );

      */
      

      // Configure the button bindings
      configureButtonBindings();
      s_Swerve.gyro.setYaw(0);

      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    private void configureButtonBindings() // TODO - Update Button Config
    {
      // Driver Buttons
      zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
      
      // Operator Buttons
      ampPreset.onTrue(new InstantCommand(() -> s_Shoulder.setAngle(Constants.ampScoreAngle)));         // Move to amp preset angle (when against amp wall)

      speakerPreset.onTrue(new InstantCommand(() -> s_Shoulder.setAngle(Constants.speakerScoreAngle))); // Move to speaker preset angle (when against sub wall)
    /*
      intakeIn.onTrue(  // Do I want to do this?  Or should this all be a command???  // TODO - Figure out what to do here
        Commands.sequence(
          new InstantCommand(() -> s_Intake.setSpeed(Constants.intakeSpeed)),
          new InstantCommand(() -> s_Shoulder.setAngle(Constants.handoffAngle)),
          new InstantCommand(() -> s_Feeder.feedUntilSeen(Constants.feederSpeed))));  // TODO - Figure out what to do here
    */
        }
    

    public void printValues()
    {
        SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
        SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch()); 
        SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
      Constants.gyroOffset = s_Swerve.gyro.getPitch();
      //s_Swerve.zeroGyro();
      s_Swerve.gyro.setYaw(180);

      return autoChooser.getSelected();
    }
}
