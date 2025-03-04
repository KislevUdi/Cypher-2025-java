package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro;
    private boolean brake = true;
    public boolean slow = false;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    
    private final SwerveDrivePoseEstimator odometer;
    private final Field2d field;
    
    public SwerveSubsystem() {
        super();    
        frontLeft = new SwerveModule("FrontLeft",
                    DriveConstants.FrontLeftDriveId,
                    DriveConstants.FrontLeftSteerId,
                    DriveConstants.FrontRighEncoderId,
                    DriveConstants.FrontLeftOffset);
    
            frontRight = new SwerveModule("FrontRight",
                    DriveConstants.FrontRightDriveMotorId,
                    DriveConstants.FrontRightSteerMotorId,
                    DriveConstants.FrontRighEncoderId,
                    DriveConstants.FrontRightOffset);
    
            backLeft = new SwerveModule("BackLeft",
                    DriveConstants.BackLeftDriveMotorId,
                    DriveConstants.BackLeftSteerMotorId,
                    DriveConstants.BackLeftEncoderId,
                    DriveConstants.BackLeftOffset);
    
            backRight = new SwerveModule("BackRight",
                    DriveConstants.BackRightMotorId,
                    DriveConstants.BackRightSteerMotorId,
                    DriveConstants.BackRightEncoderId,
                    DriveConstants.BackRightOffset);
    

            SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            };
    
            odometer = new SwerveDrivePoseEstimator(
                    DriveConstants.kDriveKinematics,
                    Rotation2d.fromDegrees(getGyroHeading()),
                    modulePositions,
                    new Pose2d(0, 0, new Rotation2d(0)));
            
            field = new Field2d();
            gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
            SmartDashboard.putData("Field Pos", field);
            SmartDashboard.putData("Swerve", this);
        }
    
        public double getVelocity() {
            ChassisSpeeds speeds = getCSpeed();
            return Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + 
                             speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
        }
    
        public void zeroHeading() {
            autoHeading(0);
        }
    
        public void autoHeading(double angle) {
            gyro.reset();
            gyro.setAngleAdjustment(angle);
            resetOdometry(getPose());
        }
    
        public double getGyroHeading() {
            double angle = gyro.getYaw();
            return 360 - angle;
        }
    
        public Rotation2d getGyroRotation2d() {
            return Rotation2d.fromDegrees(getGyroHeading());
        }
    
        public double getHeading() {
            return getRotation2d().getDegrees();
        }
    
        public Rotation2d getRotation2d() {
            return getPose().getRotation();
        }
    
        public Pose2d getPose() {
            return odometer.getEstimatedPosition();
        }
    
        public void resetOdometry(Pose2d pose) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            };
            odometer.resetPosition(getGyroRotation2d(), modulePositions, pose);
        }
    
        @Override
        public void periodic() {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            };
            odometer.update(Rotation2d.fromDegrees(getGyroHeading()), modulePositions);
            field.setRobotPose(odometer.getEstimatedPosition());
        }
    
        public void setModuleStates(SwerveModuleState[] desiredStates) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    desiredStates, DriveConstants.swerve_max_speed);
            frontLeft.setDesiredState(desiredStates[0], true);
            frontRight.setDesiredState(desiredStates[1], true);
            backLeft.setDesiredState(desiredStates[2], true);
            backRight.setDesiredState(desiredStates[3], true);
        }
    
        public void drive(double xSpeed, double ySpeed, double tSpeed, boolean fieldOriented) {
            xSpeed = Math.abs(xSpeed) > OIConstants.STICK_DEADBAND ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.STICK_DEADBAND ? ySpeed : 0.0;
            tSpeed = Math.abs(tSpeed) > OIConstants.STICK_DEADBAND ? tSpeed : 0.0;
            xSpeed *= DriveConstants.MAX_VELOCITY;
            ySpeed *= DriveConstants.MAX_VELOCITY;
            tSpeed *= DriveConstants.MAX_RAD_PER_SEC;
            if(slow) {
                xSpeed *= DriveConstants.slowDriveMultiplier;
                ySpeed *= DriveConstants.slowDriveMultiplier;
                tSpeed *= DriveConstants.slowDriveMultiplier;
            }
    
            ChassisSpeeds cSpeed;
            if (fieldOriented) {
                cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, tSpeed, getRotation2d());
            } else {
                cSpeed = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
            }
            setSpeeds(cSpeed, true);
        }
    
        public void setSpeeds(ChassisSpeeds speed, boolean correctColor) {
            if (correctColor && RobotContainer.getInstance().isRed) {
                speed.vyMetersPerSecond = -speed.vyMetersPerSecond;
                speed.vxMetersPerSecond = -speed.vxMetersPerSecond;
            }
            ChassisSpeeds temp = ChassisSpeeds.fromRobotRelativeSpeeds(
                    speed, getRotation2d());
            SwerveModuleState[] moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                    temp, new Translation2d());
            setModuleStates(moduleState);
        }
    
        public void autoDrive(ChassisSpeeds speed) {
            ChassisSpeeds temp = ChassisSpeeds.fromRobotRelativeSpeeds(speed, getRotation2d());
            SwerveModuleState[] moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                    temp, new Translation2d());
            setModuleStates(moduleState);
        }
    
        public ChassisSpeeds getCSpeed() {
            SwerveModuleState[] moduleStates = new SwerveModuleState[] {
                    frontLeft.getState(),
                    frontRight.getState(),
                    backRight.getState(),
                    backLeft.getState()
            };
            return DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStates);
        }
    
        public void change_drive(boolean switch_val) {
            // Implementation needed
        }
    
        public void resetModulesAngle() {
            frontLeft.resetToAbsolute();
            frontRight.resetToAbsolute();
            backLeft.resetToAbsolute();
            backRight.resetToAbsolute();
        }

        public void toggelSlow() {
            slow = !slow;
        }
    
        public void setBrake(boolean brake) {
            this.brake = brake;
            frontLeft.setBrake(brake);
            frontRight.setBrake(brake);
            backLeft.setBrake(brake);
            backRight.setBrake(brake);
        }
    
        public boolean getBrake() {
            return brake;
        }

        public SwerveDrivePoseEstimator getOdometer() {
            return odometer;
        }
    
    
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Gyro", this::getGyroHeading, null);
            builder.addDoubleProperty("Heading", this::getHeading, null);
            builder.addBooleanProperty("Brake", this::getBrake, this::setBrake);
            super.initSendable(builder);
        }
    }