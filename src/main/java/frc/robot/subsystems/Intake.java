package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    // Order of setpoint encoder values: Full stow, stow with algae, algae intake
    private double[] setPoints = {0, 0.2171331637, 3.474130619};

    private TalonFX intakeMotor;
    private TalonFX RollerMotor;

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


    public Intake() 
    {
        intakeMotor = new TalonFX(Constants.IntakeConstants.intakeID1);
        RollerMotor = new TalonFX(Constants.IntakeConstants.intakeID2);

        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
        RollerMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.Slot0.kP = Constants.IntakeConstants.intake_P;
        intakeConfigs.Slot0.kI = Constants.IntakeConstants.intake_I;
        intakeConfigs.Slot0.kD = Constants.IntakeConstants.intake_D; 

        intakeConfigs.CurrentLimits.withStatorCurrentLimit(35);
        intakeConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        TalonFXConfiguration RollerConfigs = new TalonFXConfiguration();
        RollerConfigs.Voltage.withPeakForwardVoltage(8);
        RollerConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(20)
            .withPeakReverseTorqueCurrent(-20);

        intakeMotor.getConfigurator().apply(intakeConfigs);
        RollerMotor.getConfigurator().apply(RollerConfigs);
    }
    
    public void setRollers(double OutputPercent)
    {
      OutputPercent /= 100.;
      RollerMotor.set(-OutputPercent);
    }

    public void toSetpoint(int setpointIndex)
    {
        intakeMotor.setControl(m_request.withPosition(setPoints[setpointIndex]).withSlot(0));
    }
}
