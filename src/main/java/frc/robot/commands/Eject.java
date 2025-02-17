package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class Eject extends Command 
{
    Claw CLAW;

    public Eject(Claw claw) 
    {
        this.CLAW = claw;
        addRequirements(claw);
    }

    public void initialize()
    {
        CLAW.spinRollers(80);
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
            return true;
        }
        return false;
    }
}