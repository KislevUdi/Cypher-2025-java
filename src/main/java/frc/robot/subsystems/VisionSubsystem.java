package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Constants.LimeLightConstants;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable ntTable;
    private final Field2d field2d = new Field2d();
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<Double> getVelocity;
    private int validCount = 0;

    public VisionSubsystem(SwerveDrivePoseEstimator poseEstimator, Supplier<Double> getVelocity) {
        this.ntTable = NetworkTableInstance.getDefault().getTable(LimeLightConstants.limelight_name);
        this.poseEstimator = poseEstimator;
        this.getVelocity = getVelocity;
        SmartDashboard.putData("Vision/field", field2d);
        SmartDashboard.putData("Vision", this);
    }

    public int id() {
        return (int) ntTable.getEntry("tid").getDouble(-1);
    }

    public boolean inView() {
        return ntTable.getEntry("tv").getDouble(0) == 1;
    }

    public int getTagId() {
        if (inView() && validCount > 2) {
            return id();
        }
        return -1;
    }

    public Pose2dWithLatency getPose() {
        if (inView() && Math.abs(getVelocity.get()) < 0.2) {
            // data - x,y,z,roll,pitch,yaw,latency,tag count, tag span, avg tag distance, avg tag area
            double[] data = ntTable.getEntry("botpose_wpiblue").getDoubleArray(new double[]{999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999});
            if (data[0] != 999) {
                return new Pose2dWithLatency(
                    new Pose2d(new Translation2d(data[0], data[1]), Rotation2d.fromDegrees(data[5])), 
                    data[6]);
            }
        }
        return null;
    }

    @Override
    public void periodic() {
        Pose2dWithLatency poseData = getPose();
        if (poseData != null) {
            field2d.setRobotPose(poseData.pose);
            validCount = validCount + 1;
            if (validCount > 2) {
                poseEstimator.addVisionMeasurement(poseData.pose, poseData.latency);
            }
        } else {
            validCount = 0;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("InView", this::inView, null);
        builder.addDoubleProperty("valid count", () -> (double) validCount, null);
        builder.addDoubleProperty("tag", () -> (double) getTagId(), null);
    }

    // Helper class to return pose with latency
    public static class Pose2dWithLatency {
        public final Pose2d pose;
        public final double latency;

        public Pose2dWithLatency(Pose2d pose, double latency) {
            this.pose = pose;
            this.latency = latency;
        }
    }
}