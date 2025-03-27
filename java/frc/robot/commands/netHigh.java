package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class netHigh extends Command 
{
    Claw CLAW;
    Elevator ELEVATOR;

    public netHigh(Claw claw, Elevator elevator) 
    {
        this.CLAW = claw;
        this.ELEVATOR = elevator;

        addRequirements(claw);
        addRequirements(elevator);

    }

    public void initialize()
    {
        ELEVATOR.changeStage(14);
    }

    public void execute()
    {
        isFinished();
    }

    public boolean isFinished()
    {

        if (ELEVATOR.getPosition() > 35)
        {
            CLAW.toPosition(6);
            return true;
        }
        
        return false;
    }
}