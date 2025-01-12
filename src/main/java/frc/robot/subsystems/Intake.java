package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor1;
    private TalonFX intakeMotor2;

    public Intake() 
    {
        intakeMotor1 = new Spark(Constants.IntakeConstants.intake_ID2, MotorType.kBrushless);
        intakeMotor2 = new Spark(Constants.IntakeConstants.intake_ID, MotorType.kBrushless);
    }
}