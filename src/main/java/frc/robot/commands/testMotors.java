package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class testMotors extends Command 
{
    Elevator ELEVATOR;
    Claw CLAW;
    Boolean TEST_ELEVATOR;
    Boolean RESET;
    Boolean REVERSE;

    public testMotors (Elevator elevator, Claw claw, boolean testingElevator, boolean reset, boolean reverse)
    {
        this.ELEVATOR = elevator;
        this.CLAW = claw;
        this.TEST_ELEVATOR = testingElevator;
        this.RESET = reset;
        this.REVERSE = reverse;
        
        addRequirements(elevator);
        addRequirements(claw);
    }

    public void initialize()
    {    
        if (!TEST_ELEVATOR && RESET)
        {
            CLAW.spinRollers(0);
            isFinished();
        }
        if (TEST_ELEVATOR && !RESET)
        {
            ELEVATOR.changeStage(3);
        }
        if (TEST_ELEVATOR && RESET)
        {
            ELEVATOR.changeStage(0);
            isFinished();
        }
    }

    public void execute()
    {
        if (!TEST_ELEVATOR && !RESET)
        {
            if (REVERSE)
            {
                CLAW.spinRollers(-50);
            }
            else 
            {
                if (CLAW.pieceDetected())
                {
                    CLAW.spinRollers(0);
                }
                else
                {
                    CLAW.spinRollers(50);
                }
            }    
        }
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
