package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.SparkConfig;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    public static final CANBus rioCABbus = new CANBus("rio");
    public static final CANBus canivoreCANbus = new CANBus("canivore");
    
    public static class SystemValues {
        public static final double l2ArmAngle = 54;
        public static final double l3ArmAngle = 135;
        public static final double intakeAlgaePower = 0.35;
        public static final double specialCoralIntakePower = -0.35;
        public static final double outputAlgaePower = -1;
        public static final double intakeCoralPower = 0.5;
        public static final double outputCoralPower = -0.2;
        public static final double intakeCoralArmAngle = 45.5;
        public static final double specialCoralIntakeArmAngle = 73;
        public static final double pickAlgaeArmAngle = 71;
        public static final double ouputAlgaeArmAngle = 20;
    }

    public static class AlgaeIntakeConstants {
        public static final TalonConfig motorConfig = new TalonConfig(33, rioCABbus, "Alga Intake")
            .withBrake(true)
            .withRampTime(0.1)
            .withCurrent(30, 30, 0)
            .withVolts(6,-6);
        public static final int LIMIT_PORT = 2;
        public static final int MIN_VELOCITY = 10;
    }

    public static class CoralIntakeConstants {
        public static final TalonConfig motorConfig = new TalonConfig(22, rioCABbus, "Coral Intake")
            .withBrake(true)
            .withRampTime(0.1)
            .withCurrent(30, 30, 0)
            .withVolts(6,-6);
            public static final int MIN_VELOCITY = 10;
        }

    public static class CoralArmlConstants {
        public static final TalonConfig motorConfig = new TalonConfig(21, rioCABbus, "Coral Arm")
            .withBrake(true)
            .withInvert(true)
            .withRampTime(0.1)
            .withMotorRatio(80)
            .withPID(4.8,0,0.1,0.25,0.12,0.01,0)
            .withMotionMagic(30, 60, 600);
        public static final double MAX_ERROR = 0.5; // degrees
        public static final int LIMIT_PORT = 1;
        public static final int ABS_ENCODER_PORT = 7;
        public static final double ABS_ENCODER_OFFSET = 0.582;
    }

    public static class AlgaeArmConstants {
        public static final double MAX_ERROR = 1.25; // degrees
        public static final SparkConfig motor1Config = new SparkConfig(31, "Algae Arm 1")
            .withBrake(true)
            .withInvert(true)
            .withRampTime(0.1)
            .withMotorRatio(360.0/300.0)
            .withPID(0.02,0,0.01,0)
            .withVelocity(20, 40, 0);
        public static final SparkConfig motor2Config = new SparkConfig(32, "Algae Arm 2")
            .withBrake(true)
            .withInvert(false)
            .withRampTime(0.1)
            .withMotorRatio(360.0/300.0)
            .withPID(0.02,0,0.01,0)
            .withVelocity(20, 40, 0);
        public static final int LIMIT_PORT = 5;
        public static final int ABS_ENCODER_PORT = 2;
        public static final double ABS_ENCODER_OFFSET = 0.404;
    }

    public static class LedConstants {
        public static final int LED_PORT = 4;
        public static final int LED_LENGTH = 120;
    }

    private static double inchToMeter(double inch) {
        return inch * 0.0254;
    }

    public static class LimeLightConstants {
        public static final String limelight_name = "limelight-luxo";
        public static final double BARGE_TAG_HEIGHT = inchToMeter(73.54);
        public static final double REEF_TAG_HEIGHT = inchToMeter(12.13);
        public static final double STATION_TAG_HEIGHT = inchToMeter(58.50);
        public static final double SIDE_TAG_HEIGHT = inchToMeter(51.25);
        
        // april tag data - id, x, y, direction, height
        public static final Object[][] april_tag_data = {
            {1, inchToMeter(657.37), inchToMeter(25.80), 126, STATION_TAG_HEIGHT},
            {2, inchToMeter(657.37), inchToMeter(291.20), 234, STATION_TAG_HEIGHT},
            {3, inchToMeter(455.15), inchToMeter(317.15), 270, SIDE_TAG_HEIGHT},
            {4, inchToMeter(365.20), inchToMeter(241.64), 0, BARGE_TAG_HEIGHT},
            {5, inchToMeter(365.20), inchToMeter(75.39), 0, BARGE_TAG_HEIGHT},
            {6, inchToMeter(530.49), inchToMeter(130.17), 300, REEF_TAG_HEIGHT},
            {7, inchToMeter(546.87), inchToMeter(158.50), 0, REEF_TAG_HEIGHT},
            {8, inchToMeter(530.49), inchToMeter(186.83), 60, REEF_TAG_HEIGHT},
            {9, inchToMeter(497.77), inchToMeter(186.83), 120, REEF_TAG_HEIGHT},
            {10, inchToMeter(481.39), inchToMeter(158.50), 180, REEF_TAG_HEIGHT},
            {11, inchToMeter(497.77), inchToMeter(130.17), 240, REEF_TAG_HEIGHT},
            {12, inchToMeter(33.51), inchToMeter(25.80), 54, STATION_TAG_HEIGHT},
            {13, inchToMeter(33.51), inchToMeter(291.20), 306, STATION_TAG_HEIGHT},
            {14, inchToMeter(325.68), inchToMeter(241.64), 180, BARGE_TAG_HEIGHT},
            {15, inchToMeter(325.68), inchToMeter(75.39), 180, BARGE_TAG_HEIGHT},
            {16, inchToMeter(235.73), inchToMeter(-0.15), 90, SIDE_TAG_HEIGHT},
            {17, inchToMeter(160.39), inchToMeter(130.17), 240, REEF_TAG_HEIGHT},
            {18, inchToMeter(144.00), inchToMeter(158.50), 180, REEF_TAG_HEIGHT},
            {19, inchToMeter(160.39), inchToMeter(186.83), 120, REEF_TAG_HEIGHT},
            {20, inchToMeter(193.10), inchToMeter(186.83), 60, REEF_TAG_HEIGHT},
            {21, inchToMeter(209.49), inchToMeter(158.50), 0, REEF_TAG_HEIGHT},
            {22, inchToMeter(193.10), inchToMeter(130.17), 300, REEF_TAG_HEIGHT}
        };

        public static final double LEFT_L3_OFFSET = 0.17;
        public static final double RIGHT_L3_OFFSET = LEFT_L3_OFFSET - 0.33;
        public static final double BACK_L3_OFFSET = -0.43;
        
        public static Translation2d getTagTranslation(int tagId) {
            return new Translation2d((double)april_tag_data[tagId - 1][1], 
                                   (double)april_tag_data[tagId - 1][2]);
        }

        public static double getTagAngle(int tagId) {
            return (double)april_tag_data[tagId - 1][3];
        }

        public static Pose2d getTagPose(int tagId) {
            return new Pose2d(getTagTranslation(tagId),
                           Rotation2d.fromDegrees(getTagAngle(tagId)));
        }

        public static Pose2d getTagRelativePosition(int tagId, double x, double y) {
            Translation2d tagTranslation = getTagTranslation(tagId);
            double angle = getTagAngle(tagId);
            Translation2d r = new Translation2d(x, y).rotateBy(Rotation2d.fromDegrees(angle));
            return new Pose2d(tagTranslation.plus(r), 
                           Rotation2d.fromDegrees(inputModulus(180 + angle, -180, 180)));
        }

        public static Pose2d getLeftL3Position(int tagId) {
            return getTagRelativePosition(tagId, BACK_L3_OFFSET, LEFT_L3_OFFSET);
        }

        public static Pose2d getRightL3Position(int tagId) {
            return getTagRelativePosition(tagId, BACK_L3_OFFSET, RIGHT_L3_OFFSET);
        }
        
        // Helper method to simulate Python's inputModulus
        private static double inputModulus(double input, double minimumInput, double maximumInput) {
            double modulus = maximumInput - minimumInput;
            double result = input % modulus;
            
            while (result >= maximumInput) {
                result -= modulus;
            }
            
            while (result < minimumInput) {
                result += modulus;
            }
            
            return result;
        }
    }

    public static class ModuleConstants {

        public static final double driveKS = 0.56548;
        public static final double driveKV = 3.7091;
        public static final double driveKA = 0.85702;

        public static final double kWheelDiameterMeters = 0.095;
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 150.0 / 7.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        
        // Drive Motor Conversion Factors 
        public static final double kPTurning = 0.2;

        public static final SparkConfig BaseSteerConig = new SparkConfig(0, "Base Steer")
            .withBrake(true)
            .withInvert(false)
            .withRampTime(0.1)
            .withMotorRatio(360.0 / kTurningMotorGearRatio)
            .withPID(0.01,0,0.0 ,0);

        public static final SparkConfig BaseDriveConig = new SparkConfig(0, "Base Drive")
            .withBrake(true)
            .withInvert(true)
            .withRampTime(0.1)
            .withMotorRatio((kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio)
            .withPID(1,0,0.0 ,1)
            .withVelocity(20, 40, 0);
        public static final CancoderConfig BaseCancoderConfig = new CancoderConfig(0, rioCABbus, "Base Cancoder")
            .withInvert(true);
        
    }

    public static class DriveConstants {
        public static final double slowDriveMultiplier = 0.4;
        
        // Chassis
        public static final double kTrackWidth = 0.63;
        public static final double kWheelBase = 0.63;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );

        public static final double swerve_max_speed = 4;

        public static final double MAX_ACCELERATION = 3;
        public static final double MAX_ANGULAR_ACCELERATION = 3;

        // FrontLeft
        public static final int FrontLeftDriveId = 2;
        public static final int FrontLeftSteerId = 1;
        public static final int FrontLeftAbsoluteEncoderId = 9;
        public static final double FrontLeftOffset = -12.7;

        // FrontRight
        public static final int FrontRightDriveMotorId = 7;
        public static final int FrontRightSteerMotorId = 8;
        public static final int FrontRighEncoderId = 10;
        public static final double FrontRightOffset = -6.5;

        // BackLeft
        public static final int BackLeftDriveMotorId = 4;
        public static final int BackLeftSteerMotorId = 3;
        public static final int BackLeftEncoderId = 12;
        public static final double BackLeftOffset = -61.0;

        // BackRight
        public static final int BackRightMotorId = 5;
        public static final int BackRightSteerMotorId = 6;
        public static final int BackRightEncoderId = 11;
        public static final double BackRightOffset = 120.8;

        // Constants
        public static final double MAX_VELOCITY = 4.6;
        public static final double MAX_RAD_PER_SEC = 2 * 2 * Math.PI;

    }
    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        // stick drift values
        public static final double STICK_DEADBAND = 0.1;
    }
}
