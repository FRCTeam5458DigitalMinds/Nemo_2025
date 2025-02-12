package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ProcessorScore extends Command {

    Intake INTAKE;

    public ProcessorScore(Intake intake) {

        this.INTAKE = intake;

        addRequirements(intake);
    
    }


    public void initialize() {

       INTAKE.setRollers(-50);
    }

    public boolean isFinished() {

        INTAKE.setRollers(0);
   
        return false;
    }
}