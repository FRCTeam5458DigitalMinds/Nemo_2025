package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase
{
    // Order of setpoint encoder values: L1(placeholder), L2, L3, L4, Net scoring
    private double[] setPoints = {0, 25.28395062, 25.28395062, 21.33333333, 20.54320988}; // 0 is placeholder for L1 (NOT YET DESIGNED)

    private TalonFX clawRotate;
    private TalonFX clawSpin;
    private CANrange clawTOF;
    
    // line below potentially incorrect; poition voltage construtor parameter 
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Claw()
    {
        clawRotate = new TalonFX(Constants.ClawConstants.clawID1);
        clawSpin = new TalonFX(Constants.ClawConstants.clawID2);
        clawTOF = new CANrange(Constants.ClawConstants.canRangeID);

        clawRotate.getConfigurator().apply(new TalonFXConfiguration());
        clawSpin.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration configsRotate = new TalonFXConfiguration();
        configsRotate.Slot0.kP = Constants.ClawConstants.claw_P;
        configsRotate.Slot0.kI = Constants.ClawConstants.claw_I;
        configsRotate.Slot0.kD = Constants.ClawConstants.claw_D;

        configsRotate.CurrentLimits.withStatorCurrentLimit(35);
        configsRotate.CurrentLimits.withStatorCurrentLimitEnable(true);

        TalonFXConfiguration configsSpin = new TalonFXConfiguration();
        configsRotate.CurrentLimits.withStatorCurrentLimit(30);
        configsRotate.CurrentLimits.withStatorCurrentLimitEnable(true);

        clawRotate.getConfigurator().apply(configsRotate);
        clawSpin.getConfigurator().apply(configsSpin);
    }

    public void spinRollers(double OutputPercent)
    {
        OutputPercent /= 100.;
        clawSpin.set(-OutputPercent);
    }

    public boolean pieceDetected()
    {
        return getTOFDistance() < .05;
    }

    public double getTOFDistance()
    {
        return clawTOF.getDistance().getValueAsDouble();
    }

    public void customPosition(double setPoint)
    {
        clawRotate.setControl(m_request.withPosition(setPoint).withSlot(0));
    }

    public void toPosition(int setpointIndex) 
    {
        clawRotate.setControl(m_request.withPosition(setPoints[setpointIndex]).withSlot(0));
    }
    
    public double getPosition()
    {
        return clawRotate.getPosition().getValueAsDouble();
    }

    public void autoIntake()
    {
        if (pieceDetected())
        {
            spinRollers(0);
        }
        else
        {
            spinRollers(50);
        }
    }
}
