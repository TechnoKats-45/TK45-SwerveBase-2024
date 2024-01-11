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

import frc.robot.autos.eventMap;


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
    private final Intake s_Intake = new Intake();
    private final Arm s_Arm = new Arm();

    private final SendableChooser<Command> autoChooser;

    //public Command autoCode = Commands.sequence(new PrintCommand("no auto selected"));

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */  // For Swerve
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Variables */
    boolean driveStatus = false;
    double setPoint;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
      // Register named commands
      NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));  // TODO - Change to actual commands
      NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));  // TODO - Change to actual commands
      NamedCommands.registerCommand("print hello", Commands.print("hello"));        // TODO - Change to actual commands
      
      s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis), 
          () -> robotCentric.getAsBoolean()
        )
      );

      s_Intake.setDefaultCommand(
        new TeleopIntake(
          s_Intake,
          operator,
          driver
        )      
      );

      s_Arm.setDefaultCommand(
        new TeleopArm(
          s_Arm,
          operator,
          operator.getRawButton(XboxController.Button.kA.value),
          operator.getRawButton(XboxController.Button.kY.value),
          driver
        )
      );

      // Configure the button bindings
      configureButtonBindings();

      autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
      SmartDashboard.putData("Auto Mode", autoChooser);
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

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
      SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

      // Add a button to run pathfinding commands to SmartDashboard
      SmartDashboard.putData
      (
        "Pathfind to Pickup Pos", AutoBuilder.pathfindToPose
        (
          new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
          new PathConstraints
          (
              4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          2.0
        )
      );
      SmartDashboard.putData
      (
        "Pathfind to Scoring Pos", AutoBuilder.pathfindToPose
        (
          new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
          new PathConstraints
          (
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          0
        )
      );

      // Add a button to SmartDashboard that will create and follow an on-the-fly path
      // This example will simply move the robot 2m in the +X field direction
      SmartDashboard.putData
      (
          "On-the-fly path", Commands.runOnce(() -> 
          {
            Pose2d currentPose = s_Swerve.getPose();
        
            // The rotation component in these poses represents the direction of travel
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath
            (
              bezierPoints, 
              new PathConstraints
              (
                4.0, 4.0, 
                Units.degreesToRadians(360), Units.degreesToRadians(540)
              ),  
              new GoalEndState(0.0, currentPose.getRotation())
            );

            // Prevent this path from being flipped on the red alliance, since the given positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
          }
        )
      );
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
