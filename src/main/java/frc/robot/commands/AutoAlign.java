package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class AutoAlign extends Command {
    CommandSwerveDrivetrain DRIVETRAIN;
    Limelight LIMELIGHT;

    Pose2d TAG;
    Transform2d LEFTPOSE;
    Transform2d RIGHTPOSE;
    Pose2d TARGETPOSE;

    boolean ISLEFT;

    public AutoAlign(CommandSwerveDrivetrain drivetrain, Limelight limelight, boolean left) 
    {
        this.DRIVETRAIN = drivetrain;
        this.LIMELIGHT = limelight;
        this.ISLEFT = left;

        addRequirements(drivetrain);
        addRequirements(limelight);
    }

    public void initialize()
    {
        //find out what this does for our silly half field
        //dynamic IDs
        TAG = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(LIMELIGHT.tagID()).get().toPose2d();

        LEFTPOSE = new Transform2d(new Pose2d(), new Pose2d(0.2,-0.2, new Rotation2d()));
        RIGHTPOSE = new Transform2d(new Pose2d(), new Pose2d(-0.2,-0.2, new Rotation2d()));

        TARGETPOSE = TAG.transformBy(RIGHTPOSE);

        if (ISLEFT)
        {
            TARGETPOSE = TAG.transformBy(LEFTPOSE);
        }
        
        DRIVETRAIN.generatePath(TARGETPOSE);
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
