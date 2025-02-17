package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class TestRotation extends Command 
{
    Claw CLAW;
    Boolean RESET;

    public TestRotation(Claw claw, boolean reset)
    {
        this.CLAW = claw;
        this.RESET = reset;
        
        addRequirements(claw);
    }

    public void initialize()
    {    
        if (!RESET)
        {
            CLAW.toPosition(3);
        }
        else
        {
            CLAW.toPosition(0);
            isFinished();
        }
    }

    public void execute()
    {
        SmartDashboard.putBoolean("Piece Detected", CLAW.pieceDetected());
        SmartDashboard.putNumber("Claw Position Ticks: ", CLAW.getPosition());
        SmartDashboard.putNumber("TOF Distance", CLAW.getTOFDistance());
    }

    public boolean isFinished()
    {
        return true;
    }
}
