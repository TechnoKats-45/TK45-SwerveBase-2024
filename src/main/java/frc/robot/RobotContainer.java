package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Auto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
    private final Climber s_Climber = new Climber();

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
      /*
      // TODO - add registered commands here
      NamedCommands.registerCommand("AutoFeed", new AutoFeed(s_Intake, s_Shoulder, s_Feeder));
      NamedCommands.registerCommand("AutoIntake", new AutoIntake(s_Intake, s_Feeder).until(() -> s_Feeder.detectGamePiece()));
      */

      s_Swerve.setDefaultCommand
      (
        new TeleopSwerve(s_Swerve, driver)
      );

      s_Intake.setDefaultCommand
      (
        new TeleopIntake(s_Intake)
      );

      s_Feeder.setDefaultCommand
      (
        new TeleopFeeder(s_Feeder)
      );
      
      s_Shoulder.setDefaultCommand
      (
        new TeleopShoulder(s_Shoulder)
      );
      
      s_Shooter.setDefaultCommand
      (
        new TeleopShooter(s_Shooter)
      );

      s_Climber.setDefaultCommand
      (
        new TeleopClimber(s_Climber)
      );

      /*
      s_Limelight.setDefaultCommand
      (
        new TeleopLimelight(s_Limelight)
      );
      */
      
      // Configure the button bindings
      configureButtonBindings();
      s_Swerve.gyro.setYaw(0);

      chooser = AutoBuilder.buildAutoChooser("Test");
      SmartDashboard.putData("Auto Choices", chooser);
    }




    
    private void configureButtonBindings()
    {
      // Driver Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Right Trigger - Automatic Fire
      driver.rightTrigger().whileTrue
      (
        new AutoFire(s_Feeder, s_Limelight, s_Shoulder) // TODO - update this ???
      );
    
      // Left Trigger - Automatic Aim (X and Y), and spin up shooter
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
        new InstantCommand(() -> s_Shooter.runShooter(Constants.Shooter.shooterSpeed))
      ));

      // Right Bumper - Automatic Intake, Feed, and Shoulder Angle
      driver.rightBumper().onTrue // THIS WORKS
      (
        Commands.sequence
        (
          new AutoIntake(s_Intake, s_Feeder, s_Shoulder).until(s_Intake::detectGamePiece), // Also sets shoulder angle
          new AutoFeed(s_Intake, s_Feeder, s_Shoulder)    // Also holds shoulder angle  
        )
      );

      // Left Bumper - Feeder Shoot
      driver.leftBumper().whileTrue(Commands.sequence
        (
          new AutoAmp(s_Feeder, s_Shoulder)
        )
      );
      
      // B Button - Zero Gyro // THIS WORKS
      driver.b().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); 

      // Start Button - Cancel All Commands // THIS WORKS // TODO - add PID Cancel
      driver.start().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

      // D-Pad Up - Climbers Up
      driver.povUp().onTrue(new InstantCommand(() -> s_Climber.setTargetHeight(Constants.Climber.climberMaxHeight)));

      // D-Pad Down - Climbers Down
      driver.povDown().onTrue(new InstantCommand(() -> s_Climber.setTargetHeight(Constants.Climber.climberMinHeight)));


      
      // Operator Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Y - Speaker Preset X
      operator.y().onTrue(new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.ampScoreAngle)));    // Move to speaker preset angle (when against sub wall)

      // A - Amp Preset X
      operator.a().onTrue(new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.handoffAngle)));     // Move to amp preset angle (when against amp wall)

      // Right Trigger - Spin Shooter
      operator.rightTrigger().whileTrue(new RunCommand(() -> s_Shooter.runShooter(Constants.Shooter.shooterSpeed)));

      // Left Bumper - Manual Intake IN
      operator.leftBumper().whileTrue(new RunCommand(() -> s_Intake.runIntake(Constants.Intake.intakeSpeed)));

      // Right Bumper - Manual Intake OUT
      operator.rightBumper().whileTrue(new RunCommand(() -> s_Intake.runIntake(Constants.Intake.outtakeSpeed)));
      
      // Up on D-Pad - Manual Feeder IN
      operator.povUp().whileTrue(new RunCommand(() -> s_Feeder.runFeeder(Constants.Feeder.handoffSpeed)));

      // Down on D-Pad - Manual Feeder OUT
      operator.povDown().whileTrue(new RunCommand(() -> s_Feeder.runFeeder(-Constants.Feeder.handoffSpeed)));

      // Left Trigger - Manual Feeder
      operator.leftTrigger().whileTrue(new RunCommand(() -> s_Feeder.runFeeder(-.5)));

      // Right Stick DOWN - Clmbers Down
      //operator.rightStick().whileTrue(new RunCommand(() -> s_Climber.manualControl(driver)));  //  TODO - need to determine target height before running

      // Right Stick UP - Climbers Up
      //operator.rightStick().whileTrue(new RunCommand(() -> s_Climber.setSpeedTarget(Constants.Climber.climberSpeed)));     //  TODO - need to determine target height before running

      // Left Stick DOWN - Arm Down
      //operator.leftStick().whileTrue(new RunCommand(() -> s_Shoulder.runShoulder(Constants.Shoulder.manualShoulderSpeed)));

      // Left Stick UP - Arm Up
      //operator.leftStick().whileTrue(new RunCommand(() -> s_Shoulder.runShoulder(-Constants.Shoulder.manualShoulderSpeed)));

      // Start Button - Cancel All Commands // TODO - add PID Cancel
      operator.start().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    }
    
    public void printValues()
    {
      //s_Climber.diagnostics();
      //s_Feeder.diagnostics();
      //s_Intake.diagnostics();
      s_Limelight.diagnostics();
      s_Shooter.diagnostics();
      s_Shoulder.diagnostics();
      //s_Swerve.diagnostics();
    }

    public void subsystemInit()
    {
      s_Shoulder.setTarget(s_Shoulder.getAngle());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
      return chooser.getSelected();
    }
}