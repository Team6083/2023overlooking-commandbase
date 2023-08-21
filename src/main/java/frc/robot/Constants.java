package frc.robot;

public final class Constants {
    public static class XboxControllerPort{
        public static final int kmain = 0;
        public static final int kvice = 1;
    }
    
    public static class DrivebaseSubConstants{
        public static final double kTrackWidthMeters = 0.53;
        public static final double ks = 0.63228;
        public static final double kv = 2.1943;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double encoderPulse = 4096;
        public static final int leftMotorID1 = 13;// MotorController ID
        public static final int leftMotorID2 = 14;
        public static final int rightMotorID1 = 11;
        public static final int rightMotorID2 = 12;
    }

    public static class JointSubConstants {
        // id
        public static final int armLeftCANId = 15;
        public static final int armRightCANId = 16;
        public static final int revEncoderChannel1 = 8;
        public static final int revEncoderChannel2 = 9;

        // const
        public static final double armEncoderPulse = 2048;
        public static final double armEncoderGearing = 198;
        public static final double armVoltLimit = 4;
        public static final double armAngleMin = -20;
        public static final double armAngleMax = 195;

        // pid
        public static final double kP = 0.3;
        public static final double[][] armAngleSetpoints = { { 35.6, 28.38, 90, -10 }, { 130, 151, 90, 180 } };
    }
}
