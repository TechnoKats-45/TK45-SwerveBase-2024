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

    public double kP = .1, kI = 0, kD = 0;
    private PIDController bottomPidController = new PIDController(kP, kI, kD);
    private final double NeoFreeSpeed = 5676; // RPM

    double target = 0;
    double speed;

    private final RelativeEncoder shooterBottomEncoder;
    private final RelativeEncoder shooterTopEncoder;

    public Shooter()
    {
        shooterBottom = new CANSparkMax(Constants.Shooter.ShooterBottomID, MotorType.kBrushless);
        shooterBottom.setSmartCurrentLimit(60);
        shooterBottom.setIdleMode(IdleMode.kBrake);
        shooterBottomEncoder = shooterBottom.getEncoder();
        shooterBottom.setInverted(false);   // TODO - maybe change this

        shooterTop = new CANSparkMax(Constants.Shooter.ShooterTopID, MotorType.kBrushless);
        shooterTop.setSmartCurrentLimit(60);
        shooterTop.setIdleMode(IdleMode.kBrake);
        shooterTopEncoder = shooterTop.getEncoder();
        shooterTop.setInverted(true);   // TODO - maybe change this
        shooterTop.follow(shooterBottom);
    }

    public double getSpeed()
    {
        return shooterBottomEncoder.getVelocity() / NeoFreeSpeed;    // Converts RPM to a percentage
    }

    public void setTarget(double speed) // Percentage
    {
        target = speed;
    }
    
    public void holdTarget() 
    {
        shooterBottom.set(target);
        //pidController.calculate(getSpeed(), target); // TODO
    }

    public void runShooter(double speed) // sets and holds target speed - OPERATOR MANUAL CONTROL
    {
        setTarget(speed);
        holdTarget();
    }

    public void diagnostics()
    {
        /*
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.add("Shooter Speed", getSpeed());
        tab.add("Shooter Target", target);
        */
        SmartDashboard.putNumber("Shooter Current", getCurrent());
        SmartDashboard.putNumber("Shooter Target", target);
        SmartDashboard.putNumber("Shoter Speed", getSpeed());
        SmartDashboard.putNumber("Shooter RPM", shooterBottomEncoder.getVelocity());
    }

    public double getCurrent()
    {
        return shooterBottom.getOutputCurrent();
    }
}
