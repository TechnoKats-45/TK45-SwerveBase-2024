package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Feeder extends SubsystemBase 
{
    private CANSparkMax feeder;
    private DigitalInput feederSensor;
    private double target;

    public Feeder()
    {
        feeder = new CANSparkMax(Constants.Feeder.FeederID, MotorType.kBrushless);
        feederSensor = new DigitalInput(Constants.Feeder.FeederSensor1Port);
        feeder.setSmartCurrentLimit(40);
    }

    public boolean detectGamePiece()   // Reads the sensor and returns true if game piece is detected
    {
        return !feederSensor.get();
    }

    public void setTarget(double speed)  // For external speed setting
    {
        target = speed;
        //SmartDashboard.putNumber("Feeder Set Speed", speed);
    }

    public void holdTarget()    // For holding the gamepiece
    {
        feeder.set(target);
        //SmartDashboard.putNumber("Feeder Target", target);
    }

    public void runFeeder(double speed) // sets and holds target speed - OPERATOR MANUAL CONTROL
    {
        setTarget(speed);
        holdTarget();
    }

    public void diagnostics()
    {
        SmartDashboard.putBoolean("Feeder Sensor", detectGamePiece());
    }
}
