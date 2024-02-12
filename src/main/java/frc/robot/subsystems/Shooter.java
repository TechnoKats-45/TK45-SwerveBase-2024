package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public class Shooter extends SubsystemBase 
{
    private CANSparkMax shooter;
    
    private SparkPIDController m_pidController;

    double target = 0;
    double speed;

    public Shooter()
    {
        shooter = new CANSparkMax(Constants.Shooter.ShooterID, MotorType.kBrushless);

        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = shooter.getPIDController();

        // set PID coefficients
        m_pidController.setP(Constants.Shooter.kP);
        m_pidController.setI(Constants.Shooter.kI);
        m_pidController.setD(Constants.Shooter.kD);
        m_pidController.setIZone(Constants.Shooter.kIz);
        m_pidController.setFF(Constants.Shooter.kFF);
        m_pidController.setOutputRange(-Constants.Shooter.kMinOutput, Constants.Shooter.kMaxOutput);
    }

    public void setSpeed(double speed)  // For external speed setting
    {
        // TODO - add safety checks
        m_pidController.setReference(speed, CANSparkMax.ControlType.kVelocity); // Sets internal PID to new velocity
        target = speed;
    }

    public void runShooter(Joystick opJoystick, Joystick drJoystick)
    {
        //if button pressed -> run shooter
        if(opJoystick.getRawButton(XboxController.Axis.kRightTrigger.value)) // TODO - Update button // Shooter button pressed  // TODO - add auto shoot checks, change button and mybe direction
        {
            shooter.set(Constants.Shooter.shooterSpeed);
        }
        else    // Shooter button not pressed
        {
            shooter.set(0);
        }
    }

    public void fireWhenReady()
    {
        // TODO - Add code to check if shooter is ready to fire
        // TODO - Add code to fire
    }
}
