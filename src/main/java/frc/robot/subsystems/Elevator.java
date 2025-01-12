package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.module.ModuleDescriptor.Exports.Modifier;

import com.revrobotics.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Elevator extends SubsystemBase {
//PLACEHOLDER VALUES
    private double[] stages = {0, 1, 2};
    private int elevatorID1 = Constants.ClimbConstants.climbID1;
    private int elevatorID2 = Constants.ClimbConstants.climbID2;

    private RelativeEncoder elevatorEncoder;
    private SparkMax elevator1;
    private SparkMax elevator2;
    private SparkClosedLoopController elevatorController;
    private SparkMaxConfig elevatorConfig1;
    private SparkMaxConfig elevatorConfig2;

    public Elevator() {
        elevator2 = new SparkMax(elevatorID2, MotorType.kBrushless);
        elevator1 = new SparkMax(elevatorID1, MotorType.kBrushless);


        elevatorConfig1
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .p(0)
                .i(0)
                .d(0)
                .outputRange(0,1);

        elevatorConfig2
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .follow(elevatorID1, true);

        elevator1.configure(elevatorConfig1,com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevatorConfig2,com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
