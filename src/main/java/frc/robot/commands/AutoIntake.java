package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class AutoIntake extends Command 
{
    private Feeder s_Feeder;
    private Intake s_Intake;
    private Shoulder s_Shoulder;
    private Shooter s_Shooter;
    private Joystick operator;
    private Joystick driver;

    /** Creates a new TeleopFeeder. */
    public AutoIntake(Feeder s_Feeder, Intake s_Intake, Shoulder s_Shoulder, Shooter s_Shooter, Joystick operator, Joystick driver) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Intake = s_Intake;
        this.s_Shoulder = s_Shoulder;
        this.s_Shooter = s_Shooter;
        this.operator = operator;
        this.driver = driver;
        
        addRequirements(s_Feeder, s_Intake, s_Shoulder, s_Shooter);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if(s_Feeder.detectGamePiece())          // Game piece in feeder - Hold
        {
            s_Feeder.setSpeed(0);               // Stop feeder
            s_Shooter.setSpeed(1);              // Spin up shooter  // TODO - move this earlier if spinup takes a while
            // Wait for shoot command
            // TODO - Add intake output slowly?  to prevent jamming
        }
        else if(s_Intake.detectGamePiece())     // Game piece in intake - Load into Feeder
        {
            // Move shoulder to set point, then...
            s_Shoulder.setAngle(Constants.handoffAngle); // TODO - Update to actual angle
            // Move game piece to feeder
            s_Intake.setSpeed(.25);           // TODO - Update to actual speed
            s_Feeder.setSpeed(.25);           // TODO - Update to actual speed
        }
        else    // Game piece not found - Intake then load into Feeder
        {
            // Run intake inwards
            s_Intake.setSpeed(1);   // Run inwards at full speed // TODO - Update to actual speed
            // TODO - Add move shoulder to set point
            s_Shoulder.setAngle(Constants.handoffAngle); // TODO - Update to actual angle        
        }
    }
}