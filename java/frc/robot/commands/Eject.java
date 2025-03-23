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
            CLAW.spinRollers(50);
        }
    }

    public void execute()
    {
        SmartDashboard.putNumber("claw spin", CLAW.getSpin());
        SmartDashboard.putNumber("claw V", CLAW.getV());

        isFinished();            
    }

    public boolean isFinished()
    {
        if (Math.abs(CLAW.getV()) < 0.05 && LEVEL != 1)
        {
            SmartDashboard.putNumber("claw V", CLAW.getV());
            CLAW.spinRollers(-100);
            SmartDashboard.putBoolean("eject done", false);

            if (CLAW.getTOFDistance() > 0.1)
            {
                SmartDashboard.putBoolean("eject done", true);
                return true;
            }
        }
        if (LEVEL == 1 && !CLAW.pieceDetected())
        {
            return true;
        }
        return false;
    }
}