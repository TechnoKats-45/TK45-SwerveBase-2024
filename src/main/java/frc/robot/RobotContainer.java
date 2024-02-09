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
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */  // For Swerve
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton drfireWhenReady = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);       // Right Trigger
    private final JoystickButton drAutoAim = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);              // Left Trigger
    private final JoystickButton drAutoIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);         // Right Bumper
    private final JoystickButton drZeroGyro = new JoystickButton(driver, XboxController.Button.kB.value);                     // B BUTTON
    private final JoystickButton drRobotCentric = new JoystickButton(driver, XboxController.Button.kX.value);                 // X BUTTON

    /* Operator Buttons */
    private final JoystickButton opIntakeIn = new JoystickButton(operator, XboxController.Button.kRightBumper.value);         // Right Bumper
    private final JoystickButton opIntakeOut = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);         // Left Bumper
    private final JoystickButton opSpeakerPreset = new JoystickButton(operator, XboxController.Button.kY.value);              // Y Button
    private final JoystickButton opManualFire = new JoystickButton(operator, XboxController.Button.kStart.value);             // Right Trigger
    private final JoystickButton opAmpPreset = new JoystickButton(operator, XboxController.Button.kA.value);                  // A Button

    /* Variables */
    boolean driveStatus = false;
    double setPoint;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
      // TODO - add registered commands here
      //NamedCommands.registerCommand("autoAimX", Commands.run(() -> s_Swerve.autoAimX()));
      NamedCommands.registerCommand("autoAimY", Commands.run(() -> s_Shoulder.autoAimY()));   
      
      s_Swerve.setDefaultCommand
      (
        new TeleopSwerve
        (
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis), 
          () -> drRobotCentric.getAsBoolean()
        )
      );
      
      s_Intake.setDefaultCommand
      (
        new TeleopIntake
        (
          s_Intake, 
          operator,
          driver
        )      
      );

      /*

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

      chooser = AutoBuilder.buildAutoChooser("Test");
      SmartDashboard.putData("Auto Choices", chooser);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    private void configureButtonBindings() // TODO - Update Button Configs
    {
      // Driver Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
         * RT - Fire
         * LT - Auto Aim
         * RB - Auto Intake
         * B - Zero Gyro
         * X - Field / Robot Centeric Toggle
         */
      drfireWhenReady.onTrue(new InstantCommand(() -> s_Shooter.fireWhenReady()));  // TODO - add this function

      drAutoAim.whileTrue(Commands.parallel
      (
        new TeleopLimelightTurret // Auto Aim X - Swerve
        (
          s_Limelight,
          s_Swerve,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          drRobotCentric
        ),
        new InstantCommand(() -> s_Shoulder.autoAimY()) // Auto Aim Y - Shoulder
      ));

      drAutoIntake.onTrue(Commands.parallel
      (
        new AutoIntake  // Enable AutoIntake
        (
          s_Intake,
          s_Shoulder,
          s_Feeder,
          s_Shooter,
          driver,
          operator
        ),
        new InstantCommand(() -> s_Shoulder.setAngle(Constants.Shoulder.handoffAngle)) // Set shoulder to handoff angle
      ));


      drZeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
      //drRobotCentric.onTrue(new InstantCommand(() -> s_Swerve.toggleRobotCentric())); // TODO - add this function
      
      // Operator Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
         * Back - Manual Mode
         * Start - Auto Mode
         * Y - Speaker Preset
         * A - Amp Preset
         * RT - Manual Fire (Manual Mode)
         * RB - Intake IN   (Manual Mode)
         * LB - Intake OUT  (Manual Mode)
         */

      // A - Amp Preset
      opAmpPreset.onTrue(new InstantCommand(() -> s_Shoulder.setAngle(Constants.Shoulder.ampScoreAngle)));         // Move to amp preset angle (when against amp wall)

      // Y - Speaker Preset
      opSpeakerPreset.onTrue(new InstantCommand(() -> s_Shoulder.setAngle(Constants.Shoulder.speakerScoreAngle))); // Move to speaker preset angle (when against sub wall)

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
        
        SmartDashboard.putBoolean("Intake GamePiece Detected", s_Intake.detectGamePiece());
        SmartDashboard.putBoolean("Feeder GamePiece Detected", s_Feeder.detectGamePiece());
        SmartDashboard.putNumber("Shoulder Angle", s_Shoulder.getAngle());

        SmartDashboard.putNumber("Limelight Updates", s_Limelight.getUpdates());
        SmartDashboard.putNumber("LimeLight X", s_Limelight.getRX());
        SmartDashboard.putNumber("LimeLight Y", s_Limelight.getRY());
        SmartDashboard.putNumber("LimeLight Z", s_Limelight.getRZ());

        SmartDashboard.putNumber("Target Detected", s_Limelight.getTV()); // If this works, then rewrite the other limelight functions following this: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
                                                                              // JTL: 2-9-24
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