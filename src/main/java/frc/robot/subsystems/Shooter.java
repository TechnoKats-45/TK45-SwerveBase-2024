package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

public class Shooter extends SubsystemBase 
{
    private CANSparkMax shooterBottom;
    private CANSparkMax shooterTop;

    public double kP = 1, kI = 0, kD = 0;
    private PIDController bottomPidController = new PIDController(kP, kI, kD);
    private PIDController topPidController = new PIDController(kP, kI, kD);
    private final double NeoFreeSpeed = 5676; // RPM

    double bottomTarget = 0;
    double topTarget = 0;
    double bottomSpeed;
    double topSpeed;

    private final RelativeEncoder shooterBottomEncoder;
    private final RelativeEncoder shooterTopEncoder;

    public Shooter()
    {
        shooterBottom = new CANSparkMax(Constants.Shooter.ShooterBottomID, MotorType.kBrushless);
        shooterBottom.setSmartCurrentLimit(60);
        shooterBottom.setIdleMode(IdleMode.kCoast);
        shooterBottomEncoder = shooterBottom.getEncoder();
        shooterBottom.setInverted(true);   // TODO - maybe change this

        shooterTop = new CANSparkMax(Constants.Shooter.ShooterTopID, MotorType.kBrushless);
        shooterTop.setSmartCurrentLimit(60);
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterTopEncoder = shooterTop.getEncoder();
        shooterTop.setInverted(true);   // TODO - maybe change this
    }

    public double getSpeedBottom()
    {
        return shooterBottomEncoder.getVelocity() / NeoFreeSpeed;    // Converts RPM to a percentage
    }

    public double getSpeedTop()
    {
        return shooterTopEncoder.getVelocity() / NeoFreeSpeed;  // Converts RPM to a percentage
    }

    public void setTarget(double speed)
    {
        bottomTarget = speed;
        topTarget = speed;
    }

    public void setBottomTarget(double bottomSpeed) // Percentage
    {
        bottomTarget = bottomSpeed;
    }

    public void setTopTarget(double topSpeed)   // Percentage
    {
        topTarget = topSpeed;
    }

    public void holdTarget() 
    {
        shooterBottom.set(bottomTarget);
        shooterTop.set(topTarget);
        //shooterBottom.set(bottomPidController.calculate(getSpeedBottom(), bottomTarget));   // TODO - test
        //shooterTop.set(topPidController.calculate(getSpeedTop(), topTarget));   // TODO - test
    }

    public void runShooter(double speed) // sets and holds target speed - OPERATOR MANUAL CONTROL
    {
        setBottomTarget(speed);
        setTopTarget(speed);
        holdTarget();
    }

    public double getBottomCurrent()
    {
        return shooterBottom.getOutputCurrent();
    }

    public double getTopCurrent()
    {
        return shooterTop.getOutputCurrent();
    }

    public void diagnostics()
    {
        /*
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.add("Shooter Speed", getSpeed());
        tab.add("Shooter Target", target);
        */
        SmartDashboard.putNumber("Bottom Shooter Current", getBottomCurrent());
        SmartDashboard.putNumber("Bottom Shooter Target", bottomTarget);
        SmartDashboard.putNumber("Bottom Shooter Speed", getSpeedBottom());
        SmartDashboard.putNumber("Bottom Shooter RPM", shooterBottomEncoder.getVelocity());

        SmartDashboard.putNumber("Top Shooter Current", getTopCurrent());
        SmartDashboard.putNumber("Top Shooter Target", topTarget);
        SmartDashboard.putNumber("Top Shooter Speed", getSpeedTop());
        SmartDashboard.putNumber("Top Shooter RPM", shooterTopEncoder.getVelocity());
    }
}
