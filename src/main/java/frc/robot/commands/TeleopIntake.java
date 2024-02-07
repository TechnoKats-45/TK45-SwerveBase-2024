// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

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

  @Override
  public boolean isFinished() 
  {
    // COMMMENTED OUT FOR TESTING OF SUBSYSTEMS // TODO
    /*
    if (s_Intake.detectGamePiece()) // Run until gamepiece detected, then stop
    {
      s_Intake.setSpeed(0);
      return true;
    }
    else
    {
      return false;
    }
  }
  */
    return false; // for test // TODO
  }
}

