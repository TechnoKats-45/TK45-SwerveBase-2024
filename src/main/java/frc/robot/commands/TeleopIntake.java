package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command 
{
  private Intake s_Intake;
  private Joystick operator;
  private Joystick driver;

  /** Creates a new TeleopIntake. */
  public TeleopIntake(Intake s_Intake, Joystick operator, Joystick driver) 
  {
    this.s_Intake = s_Intake;
    this.operator = operator;
    this.driver = driver;
    addRequirements(s_Intake);

  // Called when the command is initially scheduled.
  }

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_Intake.runIntake(operator, driver);
  }
}

