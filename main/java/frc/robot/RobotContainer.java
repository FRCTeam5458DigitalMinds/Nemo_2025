package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ReefScoring;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.lowerElevator;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.StowElevatorClaw;
import frc.robot.commands.lowerElevator;
import frc.robot.commands.resetClaw;
import frc.robot.commands.resetGyro;
import frc.robot.commands.testAutoClaw;
import frc.robot.commands.NetScore;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Eject;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

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

    private final CommandXboxController driverController = new CommandXboxController(1);
    private final CommandXboxController operatorController = new CommandXboxController(0);

    private final PIDController translatePID = new PIDController(3, 0, 0);
    private final PIDController strafePID = new PIDController(2, 0, 0);
    private final PIDController rotatePID = new PIDController(0.1, 0, 0);



    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator ELEVATOR = new Elevator();
    private final Intake INTAKE = new Intake();
    private final Claw CLAW = new Claw();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        //DRIVE COMMANDS
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-operatorController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-operatorController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-operatorController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        operatorController.leftBumper().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(translatePID.calculate(drivetrain.getPose().getX(), drivetrain.TARGETPOSELEFT.getX()))
                .withVelocityY(strafePID.calculate(drivetrain.getPose().getY(), drivetrain.TARGETPOSELEFT.getY()))
                .withRotationalRate(rotatePID.calculate(drivetrain.getYaw(), 0))
            )
        );

        operatorController.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(translatePID.calculate(drivetrain.getPose().getX(), drivetrain.TARGETPOSERIGHT.getX()))
                .withVelocityY(strafePID.calculate(drivetrain.getPose().getY(), drivetrain.TARGETPOSERIGHT.getY()))
                .withRotationalRate(rotatePID.calculate(drivetrain.getYaw(), 1))
            )
        );

        driverController.rightBumper().and(() -> drivetrain.hasValidTarget()).onTrue(
            new AutoAlign(drivetrain, false)
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        CLAW.setDefaultCommand(new testAutoClaw(CLAW));

        driverController.a().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1).andThen(new Eject(CLAW, 1)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        driverController.x().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 2).andThen(new Eject(CLAW, 2)).andThen(new lowerElevator(CLAW, ELEVATOR)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        driverController.y().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3).andThen(new Eject(CLAW, 3)).andThen(new lowerElevator(CLAW, ELEVATOR)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        driverController.b().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 4).andThen(new Eject(CLAW, 4)).andThen(new lowerElevator(CLAW, ELEVATOR)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));

        driverController.a().and(() -> !CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1).andThen(new RemoveAlgae(CLAW, 1)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        driverController.y().and(() -> !CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3).andThen(new RemoveAlgae(CLAW, 3)).andThen(new lowerElevator(CLAW, ELEVATOR)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));        

        driverController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW));

        //driverController.rightTrigger().whileTrue(new IntakeAlgae(CLAW, INTAKE));
        //driverController.povUp().onTrue(new NetScore(CLAW, INTAKE, ELEVATOR));
        //driverController.povRight().onTrue(new ProcessorScore(INTAKE));

        operatorController.a().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1).andThen(new Eject(CLAW, 1)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        //operatorController.x().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 2).andThen(new Eject(CLAW, 2)).andThen(new lowerElevator(CLAW, ELEVATOR)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        operatorController.x().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 2).andThen(new Eject(CLAW, 2)));
        operatorController.y().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3).andThen(new Eject(CLAW, 3)));
        operatorController.b().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 4).andThen(new Eject(CLAW, 4)).andThen(new lowerElevator(CLAW, ELEVATOR)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));

        operatorController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW));

        operatorController.start().onTrue(new resetGyro(drivetrain));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}