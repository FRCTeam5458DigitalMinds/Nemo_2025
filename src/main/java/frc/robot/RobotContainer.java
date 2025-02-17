package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ReefScoring;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.Lift;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.testMotors;
import frc.robot.commands.StowElevatorClaw;
import frc.robot.commands.TestRotation;
import frc.robot.commands.resetClaw;
import frc.robot.commands.testAutoClaw;
import frc.robot.commands.NetScore;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Dunk;
import frc.robot.commands.Eject;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(2);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController testController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator ELEVATOR = new Elevator();
    private final Intake INTAKE = new Intake();
    private final Claw CLAW = new Claw();
    private final Limelight LIMELIGHT = new Limelight();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        //DRIVE COMMANDS
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.leftBumper().and(() -> LIMELIGHT.hasValidTarget()).onTrue(
            new AutoAlign(drivetrain, LIMELIGHT, true)
        );

        driverController.rightBumper().and(() -> LIMELIGHT.hasValidTarget()).onTrue(
            new AutoAlign(drivetrain, LIMELIGHT, false)
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        //CLAW.setDefaultCommand(new AutoIntake(CLAW));

        driverController.a().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1).andThen(new Eject(CLAW))); 
    
        driverController.x().onTrue(new ReefScoring(CLAW, ELEVATOR, 2).andThen(new Eject(CLAW))); 
        //driverController.y().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3).andThen(new Eject(CLAW)));

        driverController.b().onTrue(new ReefScoring(CLAW, ELEVATOR, 4).andThen(new Dunk(CLAW)).andThen(new Eject(CLAW)).andThen(new Lift(CLAW)));

        driverController.a().and(() -> !CLAW.pieceDetected()).onTrue(new RemoveAlgae(CLAW, ELEVATOR, 1));
        driverController.y().and(() -> !CLAW.pieceDetected()).onTrue(new RemoveAlgae(CLAW, ELEVATOR, 3));        

        //driverController.rightTrigger().whileTrue(new IntakeAlgae(CLAW, INTAKE));
        //driverController.povUp().onTrue(new NetScore(CLAW, INTAKE, ELEVATOR));
        //driverController.povRight().onTrue(new ProcessorScore(INTAKE));
        driverController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW));

        testController.y().onTrue(new ReefScoring(CLAW, ELEVATOR, 3));
        testController.x().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1).andThen(new Eject(CLAW)));


        testController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW));


        testController.a().whileTrue(new testAutoClaw(CLAW));
        testController.a().onFalse(new resetClaw(CLAW));

        testController.b().whileTrue(new testMotors(ELEVATOR, CLAW, true, false,false));
        testController.b().onFalse(new testMotors(ELEVATOR, CLAW, true, true,false));

        //testController.x().whileTrue(new testMotors(ELEVATOR, CLAW, false, false, true));
        //testController.x().onFalse(new testMotors(ELEVATOR, CLAW, false, true,true));

        //testController.y().whileTrue(new TestRotation(CLAW, false));
        //testController.y().onFalse(new TestRotation(CLAW, true));

        //this is the operator controller, this is NOT a duplicate, but rather a 2nd controller
        //to get an operator controller either plug in 2 controllers or drag the main one to USB port #1 in the USB menu
        operatorController.a().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1)); 
        operatorController.x().onTrue(new ReefScoring(CLAW, ELEVATOR, 2)); 
        operatorController.y().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3));
        operatorController.b().onTrue(new ReefScoring(CLAW, ELEVATOR, 4)); 
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}