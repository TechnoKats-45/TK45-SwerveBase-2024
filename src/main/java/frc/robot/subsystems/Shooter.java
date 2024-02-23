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

public class Shooter extends SubsystemBase 
{
    private CANSparkMax shooter;

    public double kP = 1, kI = 0, kD = 0;
    private PIDController pidController = new PIDController(kP, kI, kD);

    double target = 0;
    double speed;

    private final RelativeEncoder shooterEncoder;

    public Shooter()
    {
        shooter = new CANSparkMax(Constants.Shooter.ShooterID, MotorType.kBrushless);
        shooter.setSmartCurrentLimit(60);
        shooterEncoder = shooter.getEncoder();
    }

    public void holdTarget() 
    {
        shooter.set(target);    //pidController.calculate(getSpeed(), target)   // IDK why this wasn't working // TODO
    }

    public void setTarget(double speed)
    {
        target = speed;
    }

    public void runShooter(double speed) // sets and holds target speed - OPERATOR MANUAL CONTROL
    {
        setTarget(speed);
        holdTarget();
        //System.out.print("runShooter Reached");
    }

    public double getSpeed()
    {
        return shooterEncoder.getVelocity();
    }

    public void fireWhenReady()
    {
        // TODO - Add code to check if shooter is ready to fire
        // TODO - Add code to fire
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
    }

    public double getCurrent()
    {
        return shooter.getOutputCurrent();
    }
}
