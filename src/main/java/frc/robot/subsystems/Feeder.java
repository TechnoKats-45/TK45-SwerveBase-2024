package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Feeder extends SubsystemBase 
{
    private CANSparkMax feeder;
    private DigitalInput FeederSensor1Port;
    private double target;

    public Feeder()
    {
        feeder = new CANSparkMax(Constants.Feeder.FeederID, MotorType.kBrushless);
        FeederSensor1Port = new DigitalInput(Constants.Feeder.FeederSensor1Port);
        feeder.setSmartCurrentLimit(40);
    }

    public boolean detectGamePiece()   // Reads the sensor and returns true if game piece is detected
    {
        return !FeederSensor1Port.get();
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
        ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
        tab.add("Feeder Sensor", detectGamePiece());
    }
}
