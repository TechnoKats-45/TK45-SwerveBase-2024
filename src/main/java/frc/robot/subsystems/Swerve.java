package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.Pigeon2;  // TODO - take back to phoenix5
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain.SimSwerveModule;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

public class Swerve extends SubsystemBase 
{
    private SimSwerveModule[] modules;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private Field2d field = new Field2d();

    public Swerve() 
    {
        configPigeon();

        mSwerveMods = new SwerveModule[] 
        {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        // Configure AutoBuilder
        AutoBuilder.configureHolonomic
        (
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            Constants.Swerve.pathFollowerConfig,
            () -> 
            {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) 
                {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) 
    {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates
            (
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds
                (
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getYaw()
                )
                : new ChassisSpeeds
                (
                    translation.getX(), 
                    translation.getY(), 
                    rotation
                )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods)
        {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Configured Pigeon IMU at startup */
    private void configPigeon()
    {        
        /* Create new Pigeon2 IMU and assign CAN ID */
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "rio");
        
        /* Configure Pigeon2 */
        var toApply = new Pigeon2Configuration();

        /* User can change the configs if they want, or leave it empty for factory-default */
        gyro.getConfigurator().apply(toApply);

        /* Speed up signals to an appropriate rate */
        gyro.getYaw().setUpdateFrequency(100);
        gyro.getGravityVectorZ().setUpdateFrequency(100);
        zeroGyro();
    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) 
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods)
        {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }    

    public ChassisSpeeds getSpeeds() 
    {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) 
    {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) 
    {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) 
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);
    
        for (int i = 0; i < modules.length; i++) 
        {
            modules[i].setTargetState(targetStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() 
    {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) 
        {
            states[i] = modules[i].getState();
        }
        return states;
    }
    
    public SwerveModulePosition[] getPositions() 
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) 
        {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
    
    /**
    * Basic simulation of a swerve module, will just hold its current state and not use any hardware
    */
    class SimSwerveModule 
    {
        private SwerveModulePosition currentPosition = new SwerveModulePosition();
        private SwerveModuleState currentState = new SwerveModuleState();
    
        public SwerveModulePosition getPosition() 
        {
            return currentPosition;
        }
    
        public SwerveModuleState getState() {
            return currentState;
        }
        public void setTargetState(SwerveModuleState targetState) 
        {
            // Optimize the state
            currentState = SwerveModuleState.optimize(targetState, currentState.angle);
            currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
        }
    }

    public Pose2d getPose() 
    {
        return swerveOdometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) 
    { 
        swerveOdometry.resetPosition(new Rotation2d(180 - gyro.getYaw().getValue(),gyro.getYaw().getValue()), getModulePositions(), pose);    // Added ".getValue()" to convert from "StatusSignalObject" to "Double" // JTL 1-11-24
    }

    public void resetOdometry(Pose2d pose) 
    {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods)
        {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro()
    {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() 
    {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(180 - gyro.getYaw().getValue()) : Rotation2d.fromDegrees(gyro.getYaw().getValue()); // Added ".getValue()" to convert from "StatusSignalObject" to "Double" // JTL 1-11-24
    }

    public void resetModulesToAbsolute()
    {
        for(SwerveModule mod : mSwerveMods)
        {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic()
    {
        swerveOdometry.update(getYaw(), getModulePositions());  
        SmartDashboard.putString("Robot Location: ", getPose().getTranslation().toString());
        SmartDashboard.putString("Yaw status", getYaw().toString());

        for(SwerveModule mod : mSwerveMods)
        {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters); 
        }
    }

    public void rotateToDegree(double target)
    {
        PIDController rotController = new PIDController(.1,0.0008,0.001);
        rotController.enableContinuousInput(-180, 180);

        double rotate = rotController.calculate(gyro.getYaw().getValue(), target); // Added ".getValue()" to convert from "StatusSignalObject" to "Double" // JTL 1-11-24

        drive(new Translation2d(0, 0), -.25*rotate, false, true);        
    }

    /**
    * Basic simulation of a gyro, will just hold its current state and not use any hardware
    */
    class SimGyro 
    {
        private Rotation2d currentRotation = new Rotation2d();

        public Rotation2d getRotation2d() 
        {
            return currentRotation;
        }
        public void updateRotation(double angularVelRps)
        {
            currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
        }
    }

    public void autoAimX()
    {
        // TODO - Write this to auto aim X based on vision
    }
}