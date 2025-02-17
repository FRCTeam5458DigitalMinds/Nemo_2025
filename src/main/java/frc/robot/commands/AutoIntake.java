package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class AutoIntake extends Command 
{
    Claw CLAW;

    public AutoIntake (Claw claw)
    {
        this.CLAW = claw;
        addRequirements(claw);
    }

    public void execute()
    {
        if (CLAW.pieceDetected())
        {
            CLAW.spinRollers(0);
        }
        else
        {
            CLAW.spinRollers(50);
        }

        SmartDashboard.putBoolean("Piece Detected", CLAW.pieceDetected());
        SmartDashboard.putNumber("TOF Distance", CLAW.getTOFDistance());
    }

    public boolean isFinished()
    {
        return true;
    }
}
