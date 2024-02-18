package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

    public void setSpeed(double speed)  // For external speed setting
    {
        // Speed Set: -1 to 1
        feeder.set(speed);  // Converts spped percent to -1 to 1 range
    }

    public void holdTarget()    // For holding the gamepiece
    {
        feeder.set(target);
    }

    public void diagnostics()
    {
        ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
        tab.add("Feeder Sensor", detectGamePiece());
    }
}
