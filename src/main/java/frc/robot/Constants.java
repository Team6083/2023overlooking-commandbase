package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;

public final class Constants {
    public static class XboxControllerPortConstants {
        public static final int kmain = 0;
        public static final int kvice = 1;
    }

    public static class DrivebaseSubConstants {
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

    public static class ArmSubConstants {
        public static final double firstStageToJoint = 50;

        public static final double extendLimit = 80; // rule: 122
        public static final double jointToFrameDist = 40;

        public static final double heightLimit = 150; // rule: 198
        public static final double jointHeight = 40; // dist from joint to floor
    }

    public static class JointSubConstants {
        // id
        public static final int jointLeftCANId = 15;
        public static final int jointRightCANId = 16;
        public static final int revEncoderChannel1 = 8;
        public static final int revEncoderChannel2 = 9;

        // const
        public static final double jointEncoderPulse = 2048;
        public static final double jointEncoderGearing = 198;
        public static final double jointVoltLimit = 4;
        public static final double jointAngleMin = -20;
        public static final double jointAngleMax = 195;
        public static final double jointInitAngleDegree = 68.5;

        // pid
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double jointCatchSetpoints[] = {-10.0, 180.0}; // { { 35.6, 28.38, 90, -10 }, { 130, 151, 90, 180 } };
        public static final double jointDoubleSubstationSetpoints[] = {35.6, 130.0};
        public static final double jointMiddleNodeSetpoints[] = {35.6, 130.0};
        public static final double jointHighNodeSetpoints[] = {35.6, 130.0};
        public static final double jointVerticalSetpoints = 90.0;

    }

    public static class LineSubConstants {

        // pid
        public static final double kP = 1.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double lineCatchSetpoints = 40.0; 
        public static final double lineDoubleSubstationSetpoints = 98.14;
        public static final double lineMiddleNodeSetpoints = 84.6;
        public static final double lineHighNodeSetpoints = 131.0;
        public static final double lineVerticalSetpoints = 40.0;

        //const
        public static final int lineId = 17;
        public static final double lineInitLength = 40;
        public static final double modifiedLineVoltPLimit = 3;
        public static final double modifiedLineVoltNLimit = -5;
        public static final double maxLineLengthLimit = 120;
        public static final double minLineLengthLimit = 40;
    }

    public static class AutoEnginePathConstants {
        public static final int trajectoryAmount = 6;
        public static final int[] blueLeft = { 0 };
        public static final int[] blueMiddle = { 1 };
        public static final int[] blueRight = { 2 };
        public static final int[] redLeft = { 3 };
        public static final int[] redMiddle = { 4 };
        public static final int[] redRight = { 5 };
        public static final String[] trajJSON = {
                "/home/lvuser/deploy/output/output/BL.wpilib.json", "/home/lvuser/deploy/output/output/BM.wpilib.json",
                "/home/lvuser/deploy/output/output/BR.wpilib.json", "/home/lvuser/deploy/output/output/RL.wpilib.json",
                "/home/lvuser/deploy/output/output/RM.wpilib.json", "/home/lvuser/deploy/output/output/RR.wpilib.json"
        };
        
        public static final double lefttWheelVoltage = 0.8;
        public static final double rightWheelVoltage = 0.8;

        public static final Trajectory[] trajectory = new Trajectory[trajectoryAmount];
        public static final String DoNothing = "DoNothing";
        public static final String BlueLeft = "BlueLeft";
        public static final String BlueMiddle = "BlueMiddle";
        public static final String BlueRight = "BlueRight";
        public static final String RedLeft = "RedLeft";
        public static final String RedMiddle = "RedMiddle";
        public static final String RedRight = "RedRight";

    }
}
