package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.SwerveModule;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Swerve extends SubsystemBase 
{
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] mSwerveMods;
    public final Pigeon2 gyro;
    final Field2d m_field = new Field2d();
    public double rotationTarget = 0;
    public Translation2d translationTarget = new Translation2d(0, 0);
    private double inchToMeterCoefficient = 0.0254;

    public Swerve() 
    {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);

        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] 
        {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry
        (
                Constants.Swerve.swerveKinematics,
                getHeading(),
                getModulePositions()
        );

        AutoBuilder.configureHolonomic
        (
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier.
                                                                                             // MUST BE ROBOT RELATIVE
                speeds -> {
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    SwerveModuleState[] swerveModuleStates =
                            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
                    setModuleStates(swerveModuleStates);
                },
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants( // Translation PID constants
                                Constants.Auto.AUTO_DRIVE_P,
                                Constants.Auto.AUTO_DRIVE_I,
                                Constants.Auto.AUTO_DRIVE_D
                        ),
                        new PIDConstants( // Rotation PID constants
                                Constants.Auto.AUTO_ANGLE_P,
                                Constants.Auto.AUTO_ANGLE_I,
                                Constants.Auto.AUTO_ANGLE_D
                        ),
                        Constants.Swerve.MAX_SPEED, // Max module speed, in m/s
                        Constants.Swerve.CENTER_TO_WHEEL, // Drive base radius in meters. Distance from robot center to
                                                          // furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() 
    {
        swerveOdometry.update(getHeading(), getModulePositions());
        m_field.setRobotPose(swerveOdometry.getPoseMeters());
    }

    public Field2d getField2d() 
    {
        return m_field;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) 
    {
        rotationTarget = rotation;
        translationTarget = translation;
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates
        (
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds
            (
                translation.getX(),
                translation.getY(),
                rotation,
                getHeading()
            ) : new ChassisSpeeds
            (
                translation.getX(),
                translation.getY(),
                rotation
            )
        );
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public double getYaw() 
    {
        return gyro.getYaw();
    }

    public Rotation2d getHeading() 
    {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getPitch() 
    {
        return gyro.getPitch();
    }

    public double getRoll() 
    {
        return gyro.getRoll();
    }

    public Pose2d getPose() 
    {
        return swerveOdometry.getPoseMeters();
    }

    public SwerveModuleState[] getModuleStates() 
    {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) 
        {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() 
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) 
        {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() 
    {
        gyro.setYaw(0);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) 
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) 
        {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) 
    {
        setModuleStates(desiredStates, true);
    }

    public void resetOdometry(Pose2d pose) 
    {
        swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void resetModulesToAbsolute() 
    {
        for (SwerveModule mod : mSwerveMods) 
        {
            mod.resetToAbsolute();
        }
    }

    public boolean isRotAligned()
    {
        if (rotationTarget <= 2)
        {
            return true;
        } 
        else 
        {
            return false;
        }
    }

    public boolean isTranslationAligned()
    {
        if(translationTarget.getX() <= (6 * inchToMeterCoefficient) && translationTarget.getY() <= (6 * inchToMeterCoefficient))  // If translation is aligned within 6 inches
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
    }
}