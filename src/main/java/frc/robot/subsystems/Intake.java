package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Intake extends SubsystemBase 
{
    private CANSparkMax intake;
    private DigitalInput intakeSensor;
    private double target;

    public Intake() 
    {
        intake = new CANSparkMax(Constants.Intake.IntakeID, MotorType.kBrushless);
        intakeSensor = new DigitalInput(Constants.Intake.IntakeSensor1Port);

        intake.setSmartCurrentLimit(40);
    }

    public boolean detectGamePiece()   // Reads the sensor and returns true if game piece is detected
    {
        return !intakeSensor.get();
    }

    public void setTarget(double speed)  // For external speed setting
    {
        target = speed;
    }

    public void holdTarget()    // For holding the gamepiece
    {
        intake.set(target);
    }

    public void intakeUntilSeen()   // Intake until gamepiece is detected
    {
        if(!intakeSensor.get())        // If gamepiece is not detected
        {
            intake.set(Constants.Intake.intakeSpeed);    // TODO - Check direction
        }
        else if(intakeSensor.get())    // If gamepiece is detected
        {
            intake.set(0);
        }
        else
        {
            // ERROR
            // TODO - add error handling / output
        }
    }

    public void diagnostics()
    {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.add("Intake Sensor", detectGamePiece());
    }
}
