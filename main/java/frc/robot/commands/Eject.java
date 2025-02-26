package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class Eject extends Command 
{
    Claw CLAW;
    int LEVEL;

    public Eject(Claw claw, int level) 
    {
        this.CLAW = claw;
        this.LEVEL = level;

        addRequirements(claw);
    }

    public void initialize()
    {
        if (LEVEL != 1)
        {
            SmartDashboard.putNumber("Claw Pos", CLAW.getPosition());
            //CLAW.toPosition(LEVEL);
        }
        else
        {
            CLAW.spinRollers(70);
        }
    }

    public void execute()
    {
        isFinished();            
    }

    public boolean isFinished()
    {
        if (Math.abs(CLAW.getV()) < 0.05 && LEVEL != 1)
        {
            CLAW.spinRollers(-50);
        }
        if (!CLAW.pieceDetected() && CLAW.getRollers() == -40)
        {
            CLAW.spinRollers(0);
            CLAW.toPosition(7);
            return true;
        }
        return false;
    }
}