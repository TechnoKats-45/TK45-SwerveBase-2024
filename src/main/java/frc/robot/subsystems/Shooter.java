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

    public double kP, kI, kD;
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
        shooter.set(pidController.calculate(getSpeed(), target));
    }

    public double getSpeed()
    {
        return shooterEncoder.getVelocity();
    }

    public void setTarget(double speed)
    {
        target = speed;
    }

    public void fireWhenReady()
    {
        // TODO - Add code to check if shooter is ready to fire
        // TODO - Add code to fire
    }

    public void diagnostics()
    {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.add("Shooter Speed", getSpeed());
        tab.add("Shooter Target", target);
    }
}
