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




    
    private void configureButtonBindings()
    {
      // Driver Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////

      // RT - Automatic Fire
      driver.rightTrigger().whileTrue(Commands.sequence
      (
        new AutoFire() // TODO - update this
      ));
    
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
        //new AutoAimY(s_Shoulder, s_Limelight),   // Auto Aim Y - Shoulder // TODO - MAKE THIS NOT PARALLEL - add to TeleopLimelightTurret?
        new InstantCommand(() -> s_Shooter.setTarget(Constants.Shooter.shooterSpeed))
      ));

      
      driver.rightBumper().onTrue // THIS does not WORK anymore // TODO - NEEDS TUNED
      (
        Commands.sequence
        (
          new AutoIntake(s_Intake, s_Feeder, s_Shoulder), // Also sets / holds shoulder angle
          new AutoFeed(s_Intake, s_Feeder, s_Shoulder)    // Also holds shoulder angle  
        )
      );
      

      /*
      driver.rightBumper().onTrue // This works
      (
        new SequentialCommandGroup
        (
          new RunCommand(() -> s_Intake.runIntake(-.75), s_Intake).until(s_Intake::detectGamePiece),
          new RunCommand(() -> s_Feeder.runFeeder(-.5), s_Feeder).until(s_Feeder::detectGamePiece)
        )
      );
      */

      
      


      driver.b().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); // THIS WORKS



      
      // Operator Buttons
      ////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Y - Speaker Preset
      operator.y().onTrue(new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.ampScoreAngle)));    // Move to speaker preset angle (when against sub wall)

      // A - Amp Preset
      operator.a().onTrue(new InstantCommand(() -> s_Shoulder.setTarget(Constants.Shoulder.handoffAngle)));     // Move to amp preset angle (when against amp wall)

      // RT - Speaker and Amp Shoot (Depending on angle)
      operator.rightTrigger().whileTrue(new RunCommand(() -> s_Shooter.runShooter(Constants.Shooter.shooterSpeed)));

      // LB - Manual Intake
      operator.leftBumper().whileTrue(new RunCommand(() -> s_Intake.runIntake(Constants.Intake.intakeSpeed)));

      // RB - Manual Feeder
      operator.rightBumper().whileTrue(new RunCommand(() -> s_Feeder.runFeeder(Constants.Feeder.handoffSpeed)));

      // LT - Manual Feeder
      operator.leftTrigger().whileTrue(new RunCommand(() -> s_Feeder.runFeeder(-.5)));

      // Down on D-Pad - Clmbers Down
      operator.povDown().whileTrue(new InstantCommand(() -> s_Climber.setTarget(10)));

      // Up on D-Pad - Climbers Up
      operator.povUp().whileTrue(new InstantCommand(() -> s_Climber.setTarget(0)));
    }
    
    public void printValues()
    {
      //s_Climber.diagnostics();
      //s_Feeder.diagnostics();
      s_Intake.diagnostics();
      //s_Limelight.diagnostics();
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