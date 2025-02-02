package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ReefScoring extends Command 
{
    Claw CLAW;
    Elevator ELEVATOR;
    int LEVEL;

    public ReefScoring(Claw claw, Elevator elevator, int level) 
    {
        this.CLAW = claw;
        this.LEVEL = level;

        addRequirements(claw);
        addRequirements(elevator);

    }

    //Initializes stuff 
    public void initialize()
    {
        CLAW.toPosition(1);        
    }

    public void execute()
    {
        if (CLAW.getPosition() < 20)
        {
            ELEVATOR.changeStage(LEVEL);
            //drive conditional here
        }
    }
}