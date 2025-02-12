package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class RemoveAlgae extends Command 
{
    private final Claw CLAW;
    private final Elevator ELEVATOR;
    private final int LEVEL;

    public RemoveAlgae(Claw claw, Elevator elevator, int level)
    {
        this.CLAW = claw;
        this.ELEVATOR = elevator;
        this.LEVEL = level;

        addRequirements(claw);
        addRequirements(elevator);
    }

    public void initialize()
    {
        CLAW.toPosition(1);
    }

    public void execute()
    {
        if (CLAW.getPosition() > 20)
        {
            CLAW.spinRollers(-50);
            ELEVATOR.changeStage(LEVEL);
            isFinished();
        }
    }

    public boolean isFinished()
    {
        return true;
    }
}
