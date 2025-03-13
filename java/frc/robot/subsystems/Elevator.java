package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    //Order of setpoint encoder values: 0, L1, L2, L3, L4, Net scoring
    //48-3.75 == 44.25
    //48-9.12 = 39

    private double[] stages = {0, 0, 6.75, 19, 41.5, 0};
    private double errorRange = 1.25;
    
    private int elevatorID1 = Constants.ClimbConstants.climbID1;
    private int elevatorID2 = Constants.ClimbConstants.climbID2;

    private SparkMax elevator1;
    private SparkMax elevator2;
    
    private SparkMaxConfig elevatorConfig1;
    private SparkMaxConfig elevatorConfig2;

    private SparkClosedLoopController elevatorController;

    public Elevator() {
        elevator2 = new SparkMax(elevatorID2, MotorType.kBrushless);
        elevator1 = new SparkMax(elevatorID1, MotorType.kBrushless);

        elevatorConfig1 = new SparkMaxConfig();
        elevatorConfig2 = new SparkMaxConfig();


        elevatorController = elevator1.getClosedLoopController();

        //need not apply PIDF to motor2, as it will follow leader motor1
        elevatorConfig1
            .smartCurrentLimit(60)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            //init-ing pidf values (change thru constants file)
            .closedLoop
                .pidf
                (
                    Constants.ClimbConstants.climb_P, 
                    Constants.ClimbConstants.climb_I, 
                    Constants.ClimbConstants.climb_D, 
                    Constants.ClimbConstants.climb_FF
                )
                .outputRange(-1, 1);

        //applying configs motor2, make sure this one is INVERTED (robot will break)
        elevatorConfig2
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake)
            .follow(elevatorID1, true);

        //applying configs to motors
        elevator1.configure(elevatorConfig1,com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevatorConfig2,com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void changeStage(int stageIndex) 
    {
        elevatorController.setReference(stages[stageIndex], ControlType.kPosition);
    }

    public boolean checkStage(int stage)
    {
        if (getPosition() < stages[stage] + errorRange && getPosition() > stages[stage] - errorRange)
        {
            return true;
        }

        return false;
    }

    public double getV()
    {
        return elevator1.get();
    }

    public double getPosition()
    {
        return elevator1.getEncoder().getPosition();
    }
    public boolean checkL4()
    {
        if (getPosition() > 20 && getPosition() < 21)
        {
            return true;
        }
        return false;
    }
}


