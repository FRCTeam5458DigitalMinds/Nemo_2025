package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    CommandSwerveDrivetrain DRIVETRAIN;
    boolean ISLEFT;

    private final PIDController translatePID = new PIDController(5, 0, 0.01);
    private final PIDController strafePID = new PIDController(5, 0, 0.01);
    private final PIDController rotatePID = new PIDController(0.4, 0, 0.01);

    public AutoAlign(CommandSwerveDrivetrain drivetrain, boolean left) 
    {
        this.DRIVETRAIN = drivetrain;
        this.ISLEFT = left;

        addRequirements(drivetrain);
    }

    public void execute()
    {
      
    }

    public boolean isFinished()
    {
        return true;
    }
}
