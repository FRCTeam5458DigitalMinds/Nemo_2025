// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ReefScoring;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.RemoveAlgae;

import frc.robot.commands.StowElevatorClaw;
import frc.robot.commands.Test;
import frc.robot.commands.NetScore;
import frc.robot.commands.ProcessorScore;
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
            
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final PIDController drivePID = new PIDController(0.1, 0, 0.);
    private final PIDController strafePID = new PIDController(0.1, 0, 0);
    private final PIDController rotationPID = new PIDController(0.1, 0, 0);

    private final int driveOffset = 10;
    private final int angleOffset = 0;
    private final double strafeOffset = 6.5;

    private final Elevator ELEVATOR = new Elevator();
    private final Intake INTAKE = new Intake();
    private final Claw CLAW = new Claw();
    private final Limelight LIMELIGHT = new Limelight();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //CLAW.setDefaultCommand(autoIntake());
        /*
        driverController.leftBumper().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(drivePID.calculate(driveOffset, driveOffset))
                .withVelocityY(strafePID.calculate(driverController.getRawAxis(2) * 10, strafeOffset))
                .withRotationalDeadband(rotationPID.calculate(angleOffset, angleOffset)))
        );
        */

        //CHANGE SPECIFIC LEFT
        
        driverController.leftBumper().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-drivePID.calculate(LIMELIGHT.distToTag(), driveOffset))
                .withVelocityY(strafePID.calculate(LIMELIGHT.strafeOffset(), strafeOffset))
            )
        );
        //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //ADD AND FOR LIMELIGHT "IN POSITION"???? DRIVETRAIN LIMELIGHT CODE
        driverController.a().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1)); 
        driverController.x().onTrue(new ReefScoring(CLAW, ELEVATOR, 2)); 
        
        driverController.y().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3));
        driverController.b().onTrue(new ReefScoring(CLAW, ELEVATOR, 4)); 

        // TEST
        //driverController.b().onTrue(new Test(LIMELIGHT));

        /* duplicate?
        operatorController.a().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 1)); 
        operatorController.x().onTrue(new ReefScoring(CLAW, ELEVATOR, 2)); 
        operatorController.y().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3));
        operatorController.b().onTrue(new ReefScoring(CLAW, ELEVATOR, 4)); 
        */

        driverController.a().and(() -> !CLAW.pieceDetected()).onTrue(new RemoveAlgae(CLAW, ELEVATOR, 1));
        driverController.y().and(() -> !CLAW.pieceDetected()).onTrue(new RemoveAlgae(CLAW, ELEVATOR, 3));        

        driverController.rightTrigger().whileTrue(new IntakeAlgae(CLAW, INTAKE));
        driverController.povUp().onTrue(new NetScore(CLAW, INTAKE, ELEVATOR));
        driverController.povRight().onTrue(new ProcessorScore(INTAKE));
        driverController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW));
       
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        drivetrain.registerTelemetry(logger::telemeterize);


    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}