package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class Dunk extends Command 
{
    Claw CLAW;
    double pos;

    public Dunk(Claw claw) 
    {
        this.CLAW = claw;
        addRequirements(claw);
    }

    public void initialize()
    {
        pos = CLAW.getPosition() - 1;
        CLAW.customPosition(pos);
    }

    public void execute()
    {
        if (CLAW.checkRotation(pos))
        {
           isFinished();  
        }
         
    }
    public boolean isFinished()
    {   
        return true;
    }
}