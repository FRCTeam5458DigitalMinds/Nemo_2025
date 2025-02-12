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
        this.ELEVATOR = elevator;
        this.LEVEL = level;

        addRequirements(claw);
        addRequirements(elevator);

    }

    public void initialize()
    {
        CLAW.toPosition(1);        
    }

    public void execute()
    {
        if (CLAW.getPosition() < 20)
        {
            ELEVATOR.changeStage(LEVEL);
            isFinished();
            //drive conditional here
        }
    }

    public boolean isFinished()
    {
        return true;
    }
}