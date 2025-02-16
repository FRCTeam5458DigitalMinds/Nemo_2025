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

    public AutoAlign(CommandSwerveDrivetrain drivetrain, Limelight limelight) 
    {
        this.DRIVETRAIN = drivetrain;
        this.LIMELIGHT = limelight;

        addRequirements(drivetrain);
        addRequirements(limelight);
    }
    
    Pose2d tag = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(7).get().toPose2d();
    //Create a transform of 0.5m left, 0.5m back, and no rotation (front away from tag)
    Transform2d leftside =new Transform2d(new Pose2d(), new Pose2d(0.5,-0.5,new Rotation2d()));
    //This pose is now the desired position and orientation to drive to
    Pose2d targetPose =tag.transformBy(leftside);
}
