package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.Cancoder;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.SparkConfig;
import frc.robot.utils.SparkMotor;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final String name;
    
    private final double absoluteEncoderOffset;
    private final Cancoder absoluteEncoder;
    
    private final SparkMotor driveMotor;
    private final SparkMotor steerMotor;
    
    public SwerveModule(
            String name,
            int driveMotorId,
            int steerMotorId,
            int absoluteEncoderId,
            double absoluteEncoderOffset) {
        
        this.name = name;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoder = new Cancoder(new CancoderConfig(absoluteEncoderId, name + "/Abs Encoder", ModuleConstants.BaseCancoderConfig));
        this.driveMotor = new SparkMotor(new SparkConfig(driveMotorId, name + "/Drive", ModuleConstants.BaseDriveConig));
        this.steerMotor = new SparkMotor(new SparkConfig(steerMotorId, name + "/Steer", ModuleConstants.BaseSteerConig));
        resetToAbsolute();    
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getCurrentPosition(), getAngle());
    }

    public double getAbsEncoder() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }
    
    // Sync the internal motor encoder with the absolute encoder
    public void resetToAbsolute() {
        double absolutePosition = getAbsEncoder() - absoluteEncoderOffset;
        steerMotor.getEncoder().setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getCurrentVelocity(), getAngle());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steerMotor.getCurrentPosition());
    }

    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
        state.optimize(getAngle());
        double angle = state.angle.getDegrees();
        if (Math.abs(angle - steerMotor.getCurrentPosition()) > 0.6) {
            steerMotor.setPositionVoltage(angle);
        } else {
            steerMotor.set(0);
        }
        setSpeed(state.speedMetersPerSecond, isOpenLoop);
    }

    private void setSpeed(double speed, boolean isOpenLoop) {
        if (isOpenLoop && Math.abs(speed) <= DriveConstants.MAX_VELOCITY * 0.05) {
            driveMotor.set(0);
        } else if (isOpenLoop) {
            double percentOutput = speed > 0? speed*speed:-speed*speed;
            speed = speed > 1? 1: (speed < -1?-1 :speed);    
            driveMotor.set(percentOutput);
        } else {
            steerMotor.setVelocity(speed);
        }
    }
    
    public void setBrake(boolean brake) {
        driveMotor.setNeutralMode(brake);
        steerMotor.setNeutralMode(brake);
    }

}
