package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;


public class lowerElevator extends Command 
{
    Claw CLAW;
    Elevator ELEVATOR;
    double pos;

    public lowerElevator(Claw claw, Elevator elevator) 
    {
        this.CLAW = claw;
        this.ELEVATOR = elevator;

        addRequirements(claw);
        addRequirements(elevator);
    }

    public void execute()
    {
        isFinished();  
    }

    public boolean isFinished()
    {
        if (CLAW.getPosition() > 27)
        {
            ELEVATOR.changeStage(0);
            return true;
        }
        return false;
    }
}