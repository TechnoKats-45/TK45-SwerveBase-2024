package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Feeder extends SubsystemBase 
{
    private CANSparkMax feeder;
    private DigitalInput FeederSensor1Port;

    public Feeder()
    {
        feeder = new CANSparkMax(Constants.Feeder.FeederID, MotorType.kBrushless);
        FeederSensor1Port = new DigitalInput(Constants.Feeder.FeederSensor1Port);
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

    public void runFeeder(Joystick opJoystick, Joystick drJoystick)
    {
        //if button pressed -> run Feeder
        if(opJoystick.getRawButton(XboxController.Button.kLeftBumper.value)) // Feeder button pressed
        {
            // Run Feeder
            feeder.set(0.5);    // TODO - Check direction and speed
        }
        else    // Feeder button not pressed
        {
            // Stop Feeder
            feeder.set(0);
        }
    }
}
