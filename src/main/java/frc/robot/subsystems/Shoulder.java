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

        SmartDashboard.putNumber("LIMITED SET", limitedSpeed);
        SmartDashboard.putNumber("ARM SET", m_pidController.calculate(getAngle(), target));
        SmartDashboard.putNumber("Set Target", target);
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

    public void diagnostics()
    {
        /*
        ShuffleboardTab tab = Shuffleboard.getTab("Shoulder");
        tab.add("Shoulder Angle", getAngle());
        tab.add("Shoulder Target", target);
        */

        SmartDashboard.putNumber("Shoulder Angle", getAngle());
    }
}