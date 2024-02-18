package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class Shoulder extends SubsystemBase
{
    private CANSparkMax shoulder;

    double target = 0;
    int angle;

    public double kP, kI, kD;
    private DutyCycleEncoder m_absoluteEncoder;
    private PIDController pidController = new PIDController(kP, kI, kD);

    public Shoulder() 
    {
        shoulder = new CANSparkMax(Constants.Shoulder.ShoulderID, MotorType.kBrushless);
        shoulder.restoreFactoryDefaults();
        shoulder.setSmartCurrentLimit(40);
        shoulder.setInverted(true);
        
        m_absoluteEncoder = new DutyCycleEncoder(Constants.Shoulder.ShoulderEncoderPort);
    }

    public double getAngle()
    {
        return m_absoluteEncoder.getAbsolutePosition() * 360;
    }

    public void moveAngle(Joystick opJoystick, Joystick drJoystick) // For manual control
    {
        holdTarget();   // Hold target angle // Button readings should happen in RobotContainer
    }

    public void holdTarget() 
    {
        shoulder.set(pidController.calculate(getAngle(), target));
    }

    public void setAlignedAngle(double x, double z, boolean tag)
    {
        //double dist = Math.hypot(x, z); // Calculates direct line distance from target
        if(tag) // If tag exists
        {
            target = Math.tan(z/x);    // Calculates angle to target based on X and Z Tangent (Opposite over Adjacent)
        }
    }

    public boolean isAligned()
    {
        if (Math.abs(getAngle() - target) <= 1) // If within 1 degree of target
        {
            return true;
        } 
        else 
        {
            return false;
        }
    }

    public void setTarget(double setPoint)  // Assigns a new target angle
    {
        target = setPoint;
    }
}