package frc.robot;

public final class Constants {
    public static final class ClimbConstants{
        public static final int climbID1 = 10;
        public static final int climbID2 = 11;

        //not final; change before running
        public static final double climb_P = 0;
        public static final double climb_I = 0;
        public static final double climb_D = 0;
        public static final double climb_FF = 0;
    }
    public static final class IntakeConstants{
        public static final int intakeID1 = 13;
        public static final int intakeID2 = 14;

        //not final; change before running
        public static final double intake_P = 0;
        public static final double intake_I = 0;
        public static final double intake_D = 0;
        public static final double intake_FF = 0;
    }
    public static final class ClawConstants{
        public static final int clawID1 = 15;
        public static final int clawID2 = 16;
        public static final int canRangeID = 12;

        //not final; change before running
        public static final double claw_P = 0;
        public static final double claw_I = 0;
        public static final double claw_D = 0;
    }

    public static final class LimelightConstants{

        //given in radians, inches respectively

        //32,6 deg op 2
        public static final double limelightMountingAngle = .546; 
        public static final double limelightMountingHeight = 6.3; 
        
    }

}
