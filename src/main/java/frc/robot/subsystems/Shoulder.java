package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Shoulder extends SubsystemBase
{
    private CANSparkMax shoulder;

    double target = 0;
    int angle;

    public double kP = .01, kI = 0.0001, kD = 0, kS, kG, kV, kA, feedForward;
    private double position = 0, velocity = 0, acceleration = 0;
    private DutyCycleEncoder m_absoluteEncoder;
    private PIDController m_pidController;
    private ArmFeedforward m_feedforward;


    public Shoulder() 
    {
        shoulder = new CANSparkMax(Constants.Shoulder.ShoulderID, MotorType.kBrushless);
        shoulder.restoreFactoryDefaults();
        shoulder.setSmartCurrentLimit(40);
        shoulder.setInverted(true);

        m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
        m_pidController = new PIDController(kP, kI, kD);

        m_absoluteEncoder = new DutyCycleEncoder(Constants.Shoulder.ShoulderEncoderPort);

        target = getAngle();    // TODO - change to handoff angle eventually
    }

    public double getAngle()
    {
        return m_absoluteEncoder.getAbsolutePosition() * 360;
    }

    public void moveAngle(Joystick opJoystick, Joystick drJoystick) 
    {
        holdTarget();   // Hold target angle // Button readings should happen in RobotContainer
    }

    public void holdTarget() 
    {
        //feedForward = m_feedforward.calculate(position, velocity, acceleration);
        //shoulder.set((m_pidController.calculate(getAngle(), target))); // add back in "+ feedforward"   // removed  * Constants.Shoulder.speedMultiplier
        //SmartDashboard.putNumber("Arm Set", ((m_pidController.calculate(m_absoluteEncoder.getAbsolutePosition(), target)) * Constants.Shoulder.speedMultiplier));

        double pidOutput = m_pidController.calculate(getAngle(), target);
        double maxSpeed = 0.75; // Example maximum speed value. Adjust this to your needs.  // TODO - use constant

        // Limit the speed to the range [-maxSpeed, maxSpeed]
        double limitedSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, pidOutput));

        // Set the motor speed with the limited value
        shoulder.set(limitedSpeed);
    }

    public void setAlignedAngle(double x, double z, boolean tag)
    {
        if(tag) // If tag exists
        {
            target = Math.hypot(z, x) - Constants.Shoulder.groundParallelAngle;    // Calculates direct line to taget angle based on X and Z (Hypotenuse)
        }
    }

    public boolean isAligned()
    {
        if (Math.abs(getAngle() - target) <= 1) // If within 1 degree of target
        {
            // DO NOT PUT A PRINT STATEMENT HERE! - THIS WILL BREAK EVERYTHING... again
            return true;    
        } 
        else 
        {
            return false;
        }
    }

    public void setTarget(double setPoint)  // Assigns a new target angle
    {
        /*
        // Checks to make sure angle is within limits
        if(setPoint > Constants.Shoulder.maxAngle)
        {
            setPoint = Constants.Shoulder.maxAngle;
        }
        else if(setPoint < Constants.Shoulder.minAngle)
        {
            setPoint = Constants.Shoulder.minAngle;
        }
        */
        target = setPoint;
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Shoulder Angle", getAngle());
        SmartDashboard.putNumber("Shoulder Targer", target);
    }
}