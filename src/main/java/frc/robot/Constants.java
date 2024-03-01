package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants 
{
    public static final double STICK_DEADBAND = 0.1;

    public static double MINIMUM_ANGLE = -180;
    public static double MAXIMUM_ANGLE = 180;

    /* Vision Constants */
    public static final class AprilTags
    {
        public static final double speakerHeightOffset = Units.inchesToMeters(24.25); // 78.13 - 53.88;
        public static final double ampHeightOffset = 0;
    }

    public static final class Limelight     // TODO - TUNE THIS
    {
        public static final double LIMELIGHT_P = 0.02;
        public static final double LIMELIGHT_I = 0;
        public static final double LIMELIGHT_D = 0.0001;    // Was 0
        public static final double YAW_OFFSET = 0;
        public static final double HEIGHT_OFFSET = 2; // TODO - Update this value
        public static final double X_Alignment_Tolerance = 3;   // TODO - Update this value
        public static final double PITCH_OFFSET = 35;
        public static final int LED_OFF = 1;
        public static final int LED_BLINK = 2;
        public static final int LED_ON = 3;
    }

    /* Intake Constants */
    public static final class Intake 
    {
        public static final int IntakeID = 40;
        public static final double intakeSpeed = -.5; // TODO - Adjust intake speed // Was -0.75
        public static final int IntakeSensor1Port = 1;   
        public static final double handoffSpeed = -.25; // TODO - Adjust handoff speed
        public static final double outtakeSpeed = .50; // TODO - Adjust outtake speed
    }

    /* Feeder Constants */
    public static final class Feeder 
    {
        public static final int FeederID = 50;
        public static final int FeederSensor1Port = 0;
        public static final double handoffSpeed = .40;  // TODO - Adjust feeder speed
        public static final double ampScoreSpeed = -1; // TODO - Adjust amp shoot speed
        public static final double speakerFeedSpeed = 1;
        public static final double JiggleDelay = 0.25;  // TODO - update
    }

    /* Shoulder Constants */   
    public static final class Shoulder 
    {
        public static final int ShoulderID = 60;
        public static final double handoffAngle = 28;           // TODO - Update
        public static final double ampScoreAngle = 224.8;       // TODO - Update
        public static final double speakerScoreAngle = handoffAngle;      // TODO - Update
        public static final int ShoulderEncoderPort = 2;
        public static final double maxAngle = 270;
        public static final double minAngle = handoffAngle;       // TODO - Update
        public static final double groundParallelAngle = 79;      // TODO - Update maybe
        public static final double manualShoulderSpeed = 20;        // TODO - Update
        public static final double maxSpeed = 1;  // Was 0.25
        public static final double whiteLineSpeakerPreset = 30;
    }

    /* Shooter Constants */
    public static final class Shooter 
    {
        public static final double shooterSpeed = .9; // TODO - Adjust shooter speed
        public static final int ShooterBottomID = 55;
        public static final int ShooterTopID = 56;

        // TODO - tune PID
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0; 
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = 0;
    }

    /* Climber Constants */
    public static final class Climber
    {
        public static final int ClimberID = 45;
        public static final double climberSpeed = 1;       // TODO - Adjust climber speed
        public static final double minExtension = 0;
        public static final double maxExtension = 110;     // TODO - Update this value
        public static final double chainGrabHeight = 99;   // TODO - Update this value
        public static final int ClimberEncoderPort = 3;
        public static final double ClimbSpeed = .25;       // TODO - Adjust climb speed
        public static final double climberMaxAngle = 100; // TODO - Update this value
        public static final double climberMinAngle = 0;   // TODO - Update this value
    
        // TODO - tune PID
        public static final double kP = 0.1;
        public static final double kI = 1e-4;
        public static final double kD = 1; 
        public static final double kIz = 0; 
        public static final double kFF = 0; 
        public static final double kMaxOutput = .2;     // TODO - Update this to 1 after testing
        public static final double kMinOutput = -.2;    // TODO - Update this to 1 after testing

        public static final double kInchesPerRotation = 2; // TODO - Update this value
    }

    /* Gyro Constants */
    public static double gyroOffset = 0;

    public static final class Swerve 
    {
        public static final int pigeonID = 50;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)

        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(26);
        public static final double WHEEL_BASE = Units.inchesToMeters(26);
        public static final double CENTER_TO_WHEEL = Math.sqrt(Math.pow(WHEEL_BASE / 2.0, 2) + Math.pow(TRACK_WIDTH / 2.0, 2));
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics
        (
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), 
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive NEO to ramp in open loop and closed loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.16861 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (2.6686 / 12);
        public static final double driveKA = (0.34757 / 12);

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /* Swerve Profiling Values */
        /** Meters per Second */    // Controls the translational speed and acceleration of the robot (left joystick)
        public static final double MAX_SPEED = 1;  // was 4.1
        public static final double MAX_ACCEL = 1;  // was 4.1

        /** Radians per Second */ // Controls the rotational speed of the robot (right joystick)
        public static final double maxAngularVelocity = 5; // was 10
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 
        { 
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 23;
            public static final int canCoderID = 33;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(102.48+180-0.703);      // FOR PRACTICE BOT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(6.15+180-12);  // FOR COMP BOT  // TODO - UPDATE THIS
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1
        { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 20;
            public static final int canCoderID = 30;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(25.84+180);   // PRACTICE BOT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-73.04+180);  // FOR COMP BOT  // TODO - UPDATE THIS
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 
        { 
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 32;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-50.889+180-18.281);  // PRACTICE BOT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-92.9+180);  // FOR COMP BOT  // TODO - UPDATE THIS
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 
        { 
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 31;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-170.508+180);    // PRACTICE BOT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(134.21+180);  // FOR COMP BOT  // TODO - UPDATE THIS
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig
        (
            new PIDConstants(5.0, 0, 0), // Translation constants 
            new PIDConstants(5.0, 0, 0), // Rotation constants 
            MAX_SPEED, 
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0).getNorm(), // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
        );
    }

    public static final class Auto 
    {
        public static final double AUTO_DRIVE_P = 2.5;
        public static final double AUTO_DRIVE_I = 0.0;
        public static final double AUTO_DRIVE_D = .01;
        public static final double AUTO_ANGLE_P = 2;    // Was 5    // Was 3
        public static final double AUTO_ANGLE_I = 0.0;
        public static final double AUTO_ANGLE_D = 0.0;
    }
}
