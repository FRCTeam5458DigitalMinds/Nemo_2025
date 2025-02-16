package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class TestRotation extends Command 
{
    Elevator ELEVATOR;
    Claw CLAW;
    Boolean TEST_ELEVATOR;
    Boolean RESET;
    Boolean REVERSE;

    public TestRotation(Claw claw,boolean reset)
    {
        this.CLAW = claw;
        this.RESET = reset;
        
        addRequirements(claw);
    }

    public void initialize()
    {    
        if (!RESET)
        {
            CLAW.toPosition(1);
        }
        else if (RESET)
        {
            CLAW.toPosition(0);
        }
        if (RESET)
        {
            isFinished();
        }
    }

    public void execute()
    {
        SmartDashboard.putBoolean("Piece Detected", CLAW.pieceDetected());
        SmartDashboard.putNumber("Elevator Position Ticks: ", ELEVATOR.getPosition());
        SmartDashboard.putNumber("Claw Position Ticks: ", CLAW.getPosition());
        SmartDashboard.putNumber("TOF Distance", CLAW.getTOFDistance());
    }

    public boolean isFinished()
    {
        return true;
    }
}
