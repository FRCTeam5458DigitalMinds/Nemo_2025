package frc.robot.commands;

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
            CLAW.customPosition(5);
        }
        else if (!TEST_ELEVATOR && RESET)
        {
            CLAW.spinRollers(0);
            //CLAW.toPosition(0);
        }


    }
}
