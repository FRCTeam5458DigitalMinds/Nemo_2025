package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    CommandSwerveDrivetrain DRIVETRAIN;

    Pose2d TAG;
    Transform2d LEFTPOSE;
    Transform2d RIGHTPOSE;
    Pose2d TARGETPOSE;

    boolean ISLEFT;

    public AutoAlign(CommandSwerveDrivetrain drivetrain, boolean left) 
    {
        this.DRIVETRAIN = drivetrain;
        this.ISLEFT = left;

        addRequirements(drivetrain);
    }

    public void initialize()
    {
        //find out what this does for our silly half field
        //dynamic IDs
        TAG = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(7).get().toPose2d();

        LEFTPOSE = new Transform2d(new Pose2d(), new Pose2d(1,-1, new Rotation2d()));
        RIGHTPOSE = new Transform2d(new Pose2d(), new Pose2d(-1,-1, new Rotation2d()));

        TARGETPOSE = TAG.transformBy(RIGHTPOSE);

        if (ISLEFT)
        {
            TARGETPOSE = TAG.transformBy(LEFTPOSE);
        }
        
    }

    public void execute()
    {
        isFinished();
    }

    public boolean isFinished()
    {
        return true;
    }
}
