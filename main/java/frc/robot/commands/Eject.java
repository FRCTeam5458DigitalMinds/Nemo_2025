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
        CLAW.spinRollers(-50);
    }

    public void execute()
    {
        isFinished();            
    }

    public boolean isFinished()
    {
        return true;
    }
}