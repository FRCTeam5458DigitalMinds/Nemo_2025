package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class RemoveAlgae extends Command 
{
    private final Claw CLAW;
    private final int LEVEL;

    public RemoveAlgae(Claw claw, int level)
    {
        this.CLAW = claw;
        this.LEVEL = level;

        addRequirements(claw);
    }

    public void initialize()
    {
        if (LEVEL != 1)
        {
            CLAW.toPosition(LEVEL);
        }

        CLAW.spinRollers(-80);
    }

    public void execute()
    {
        isFinished();            
    }

    public boolean isFinished()
    {
        if (!CLAW.pieceDetected())
        {
            CLAW.spinRollers(0);
            CLAW.toPosition(6);
            
            return true;
        }
        return false;
    }
}
