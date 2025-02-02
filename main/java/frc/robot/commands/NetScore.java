package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;



public class NetScore extends Command {
    
    Claw CLAW;
    Intake INTAKE;
    Elevator ELEVATOR;

    public NetScore(Claw claw, Intake intake, Elevator elevator)
    {
        this.CLAW = claw;
        this.INTAKE = intake;
        this.ELEVATOR = elevator;

        addRequirements(claw);
        addRequirements(intake);
        addRequirements(elevator);
       
    }

    

}
