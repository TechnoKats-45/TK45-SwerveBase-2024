package frc.robot.autos;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;

public class eventMap 
{
    public HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Arm s_Arm;

    public eventMap(Swerve _s_Swerve, Intake _s_Intake, Arm _s_Arm)
    {
        this.s_Swerve = _s_Swerve;
        this.s_Intake = _s_Intake;
        this.s_Arm = _s_Arm;

        eventMap.put("autoCorrect", new InstantCommand(() -> s_Swerve.rotateToDegree(180)));

        eventMap.put( // scores MID cube
            "scoreCubeMid", 
            Commands.sequence(
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_MID_FRONT_SCORE)),
            new WaitCommand(0.25),           // TK45 - May need to adjust time
            new InstantCommand(() -> s_Intake.setSpeed(-1), s_Intake),    // Spit out cube
            new WaitCommand(.5),   // TK 45 - May need to adjust time
            new InstantCommand(() -> s_Intake.setSpeed(0), s_Intake)    // Stop Intake
            //new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_LOW_FRONT_SCORE))
            )
        );

        eventMap.put( // scores LOW cube and stows arm
            "scoreCubeLow", 
            Commands.sequence(
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_LOW_FRONT_SCORE)),
            new WaitCommand(0.75), // TK45 - May need to adjust time
            new InstantCommand(() -> s_Intake.setSpeed(-1), s_Intake),    // Spit out cube // TK45 - May need to adjust speed / directon // TEST THIS
            new WaitCommand(0.5),   // TK 45 - May need to adjust time
            new InstantCommand(() -> s_Intake.setSpeed(0), s_Intake),    // Stop Intake // TK45 - May need to adjust speed / directon // TEST THIS
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_LOW_FRONT_SCORE))
            )
        );

        eventMap.put(   // Picks up CUBE from GROUND
            "pickupCubeLow",
            Commands.sequence(
                new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_LOW_FRONT_SCORE)),
                new WaitCommand(0.75),  // TK45 - May need to adjust time
                new InstantCommand(() -> s_Intake.setSpeed(.25), s_Intake)    // Suck in Cube // TK45 - May need to adjust speed / directon // TEST THIS
                // Continue to suck in while driving to hold cube in
                )
        );

        eventMap.put( // Stows Arm
            "stow", 
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_LOW_FRONT_SCORE)) 
         );
    }

    public HashMap<String, Command> getMap() 
    {
        return eventMap;
    }

}
