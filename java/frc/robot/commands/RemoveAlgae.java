package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

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
        CLAW.spinRollers(-70);
    }

    public void execute()
    {
        isFinished();            
    }

    public boolean isFinished()
    {
        SmartDashboard.putNumber("TOF", CLAW.getTOFDistance());
        if(CLAW.algaeDetected())
        {
            CLAW.spinRollers(-70);
            return true;
        }
        return false;
    }
}
