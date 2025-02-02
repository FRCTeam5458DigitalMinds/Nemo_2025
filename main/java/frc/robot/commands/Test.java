package frc.robot.commands;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Test extends Command {

    Limelight LIMELIGHT;

    public Test(Limelight Limelight)
    {
        this.LIMELIGHT = Limelight;
        
        addRequirements(Limelight);

    }

    public void initialize() {
        SmartDashboard.putNumber("Dist to Tag", LIMELIGHT.distToTag());

        isFinished();
    }

    public boolean isFinished() {
        return true;
    }

    
}
