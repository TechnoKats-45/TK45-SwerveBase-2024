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
    private final Vision s_Vision = new Vision();

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
    private final JoystickButton intakeIn = new JoystickButton(operator, XboxController.Button.kA.value);       // TODO - Update Button Config
    private final JoystickButton intakeOut = new JoystickButton(operator, XboxController.Button.kA.value);      // TODO - Update Button Config
    private final JoystickButton speakerAim = new JoystickButton(operator, XboxController.Button.kA.value);     // TODO - Update Button Config
    private final JoystickButton shooterSpin = new JoystickButton(operator, XboxController.Button.kA.value);    // TODO - Update Button Config
    private final JoystickButton ampOut = new JoystickButton(operator, XboxController.Button.kA.value);         // TODO - Update Button Config
    private final JoystickButton feederFeed = new JoystickButton(operator, XboxController.Button.kA.value);     // TODO - Update Button Config

    /* Variables */
    boolean driveStatus = false;
    double setPoint;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
      NamedCommands.registerCommand("autoIntake", Commands.run(() -> s_Intake.autoIntake()));
      NamedCommands.registerCommand("autoAimX", Commands.run(() -> s_Swerve.autoAimX()));
      NamedCommands.registerCommand("autoAimY", Commands.run(() -> s_Shoulder.autoAimY()));          
      //NamedCommands.registerCommand("fireWhenReady", Commands.run(() -> s_Shooter.fireWhenReady()));
      
      
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

      s_Vision.setDefaultCommand
      (
        new TeleopVision
        (
          s_Vision,
          operator,
          driver
        )
      );
      */

      // Configure the button bindings
      configureButtonBindings();
      s_Swerve.gyro.setYaw(0);

      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
      //Driver Buttons (and op buttons) 
      zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    public void printValues()
    {
        SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw().getValue());     // Added ".getValue()" to convert from "StatusSignalObject" to "Double" // JTL 1-11-24
        SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch().getValue()); // Added ".getValue()" to convert from "StatusSignalObject" to "Double" // JTL 1-11-24
        SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll().getValue());   // Added ".getValue()" to convert from "StatusSignalObject" to "Double" // JTL 1-11-24
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
      Constants.gyroOffset = s_Swerve.gyro.getPitch().getValue(); // Added ".getValue()" to convert from "StatusSignalObject" to "Double" // JTL 1-11-24
      //s_Swerve.zeroGyro();
      s_Swerve.gyro.setYaw(180);

      return autoChooser.getSelected();
    }
}
