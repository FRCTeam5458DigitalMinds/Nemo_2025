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
        ELEVATOR.changeStage(4);

    }

    public void execute()
    {
        isFinished();
    }

    public boolean isFinished()
    {

        if (Math.abs(ELEVATOR.getV()) < .01 && ELEVATOR.getPosition() > 39)
        {
            CLAW.toPosition(6);
            return true;
        }
        
        return false;
    }
}