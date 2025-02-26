package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if (LEVEL != 1){
            CLAW.toPosition(6);  
        }
        else {
            CLAW.toPosition(1);
        }
    }

    public void execute()
    {
        SmartDashboard.putNumber("Claw Pos", CLAW.getPosition());
        isFinished();
    }

    public boolean isFinished()
    {
        if (CLAW.getPosition() > 10 && LEVEL != 1)
        {
            ELEVATOR.changeStage(LEVEL);

            if (Math.abs(ELEVATOR.getV()) < .03)
            {
                return true;
            }
        }

        if (LEVEL == 1 && CLAW.getPosition() > 7)
        {
            return true;
        }
        
        return false;
    }
}