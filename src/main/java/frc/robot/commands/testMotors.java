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

    public testMotors (Elevator elevator, Claw claw, boolean testingElevator, boolean reset)
    {
        this.ELEVATOR = elevator;
        this.CLAW = claw;
        this.TEST_ELEVATOR = testingElevator;
        this.RESET = reset;
        
        addRequirements(elevator);
        addRequirements(claw);
    }

    public void initialize()
    {    
        if (!TEST_ELEVATOR && !RESET)
        {
            CLAW.spinRollers(50);
            //CLAW.customPosition(5);
        }
        else if (!TEST_ELEVATOR && RESET)
        {
            CLAW.spinRollers(0);
            //CLAW.toPosition(0);
        }
        else if (TEST_ELEVATOR && !RESET)
        {
            ELEVATOR.changeStage(1);
        }
        else
        {
            ELEVATOR.changeStage(0);
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
    }

    public boolean isFinished()
    {
        return true;
    }
}
