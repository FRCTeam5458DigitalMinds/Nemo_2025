package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase 
{
    double m_hasValidTarget;
    double m_x_AngleOffset;
    double m_y_AngleOffset;
    double m_areaDetected;
    long tag_ID;
    double y_AngleOffsetRads;
    double distance;
    double reefTagHeightInches = 0.307975; 
  

    public Limelight()
    {
        updateLimelightTracking();
    }

    public void updateLimelightTracking() 
    {
        m_hasValidTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        m_x_AngleOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        m_y_AngleOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        m_areaDetected = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        tag_ID =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger((0));
    }

    // need valid target conditionals??


    // MAKE WORK FOR MORE THAN JUST AUTO ADJUST

    //forward/back distance for auto adjustment
    public boolean hasValidTarget()
    {
        updateLimelightTracking();

        return m_hasValidTarget == 1;
    }
    

    public int tagID()
    {
        updateLimelightTracking();
        return ((int)tag_ID);
    }
    public double distToTag() 
    {
        updateLimelightTracking();

        //double angle = Constants.LimelightConstants.limelightMountingAngle + m_y_AngleOffset*(Math.PI/180);

        if (!hasValidTarget())
        {
            return 0;
        }

        if (hasValidTarget())
        {
            y_AngleOffsetRads = m_y_AngleOffset * (Math.PI/180);
            
            distance = ((reefTagHeightInches - Constants.LimelightConstants.limelightMountingHeight) // from mid??
            / Math.tan(Constants.LimelightConstants.limelightMountingAngle + y_AngleOffsetRads)); 
            
            // TO DO: SUBTRACT DIST FROM LENS TO BUMPER
        }
        SmartDashboard.putNumber("Dist to Tag", distance);
        //SmartDashboard.putNumber("Angle", angle);
        
        return distance; 

    }

    public double strafeOffset()// strafe distance (from limelight lens/center of robot)
    {
        updateLimelightTracking();
        
        if (!hasValidTarget())
        {
            return 0;
        }
        double distanceToTag = distToTag();

        double x_AngleOffsetRads = m_x_AngleOffset*(Math.PI/180);

        double strafe = distanceToTag * Math.tan(x_AngleOffsetRads);


        SmartDashboard.putNumber("Strafe", strafe);
       
        return strafe;
    }

    public double rotationOffset() //rotational offset
    {   
        updateLimelightTracking();

        if (!hasValidTarget())
        {
            return 0;
        }

        return m_x_AngleOffset;
    }
}





