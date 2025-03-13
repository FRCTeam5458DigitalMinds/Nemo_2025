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
        if (LEVEL < 10 && LEVEL != 1){
            CLAW.toPosition(6);  
        }
        else if (LEVEL == 1) {
            CLAW.toPosition(1);
        } else {
            CLAW.toPosition(8);
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
            if (LEVEL > 10) {
                ELEVATOR.changeStage(LEVEL - 10);
            } else {
                ELEVATOR.changeStage(LEVEL);
            }

            if (Math.abs(ELEVATOR.getV()) < .01)
            {
                if (LEVEL == 4)
                {
                    return ELEVATOR.getPosition() > 39;
                }
                SmartDashboard.putNumber("getV", ELEVATOR.getV());
                return true;
            }
        }

        if (LEVEL == 1 && CLAW.getPosition() > 12)
        {
            return true;
        }
        
        return false;
    }
}