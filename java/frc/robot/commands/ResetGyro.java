package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetGyro extends Command {
    CommandSwerveDrivetrain DRIVETRAIN;

    public ResetGyro(CommandSwerveDrivetrain drivetrain){
        this.DRIVETRAIN = drivetrain;

        addRequirements(DRIVETRAIN);
    }

    public void initialize()
    {
        DRIVETRAIN.resetPOSE();

        SmartDashboard.putString("reset", "reset");
    }
    
    public void execute()
    {
        isFinished();
    }

    @Override
    public boolean isFinished() {
        return true;
    }       
}