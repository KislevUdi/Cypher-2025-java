package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;

public class GoToPose extends Command {
    protected static double driveKp = 0.4;
    protected static double omegaKp = 0.05;

    protected SwerveSubsystem subsys;
    protected Pose2d pose;
    protected boolean targetReached;
    protected CommandXboxController controller;

    public GoToPose(Pose2d pose, SwerveSubsystem subsys) {
        this.subsys = subsys;
        this.pose = pose;
        this.controller = RobotContainer.getInstance().driverController;
        this.targetReached = false;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        if(pose != null) {
            targetReached = false;
            SmartDashboard.putNumber("Go To x", pose.getTranslation().getX());
            SmartDashboard.putNumber("Go To y", pose.getTranslation().getY());
            SmartDashboard.putNumber("Go To heading", pose.getRotation().getDegrees());
            Pose2d currentPose = subsys.getPose();
            SmartDashboard.putNumber("Go From x", currentPose.getTranslation().getX());
            SmartDashboard.putNumber("Go From y", currentPose.getTranslation().getY());
            SmartDashboard.putNumber("Go From heading", currentPose.getRotation().getDegrees());
            Translation2d errorT = pose.getTranslation().minus(currentPose.getTranslation());
            Rotation2d errorR = pose.getRotation().minus(currentPose.getRotation());
            SmartDashboard.putNumber("Go error x", errorT.getX());
            SmartDashboard.putNumber("Go error y", errorT.getY());
            SmartDashboard.putNumber("Go error heading", errorR.getDegrees());

        } else {
            targetReached = true;
            SmartDashboard.putNumber("Go To x", 999);
            SmartDashboard.putNumber("Go To y", 999);
            SmartDashboard.putNumber("Go To heading", 999);
        }
    }

    @Override
    public void execute() {
        // stop if driver is driving
        if (Math.abs(controller.getLeftY()) > 0.1 ||
            Math.abs(controller.getLeftX()) > 0.1 ||
            Math.abs(controller.getRightX()) > 0.1) {
            targetReached = true;
        } else if (pose != null) {
            Pose2d currentPose = subsys.getPose();
            Translation2d errorT = pose.getTranslation().minus(currentPose.getTranslation());
            Rotation2d errorR = pose.getRotation().minus(currentPose.getRotation());
            double x = errorT.getX();
            double y = errorT.getY();
            double deg = MathUtil.inputModulus(errorR.getDegrees(), -180,180);
            targetReached = errorT.getNorm() < 0.03 && Math.abs(deg) < 3;
            subsys.setSpeeds(new ChassisSpeeds(x * driveKp, y * driveKp, deg * omegaKp), false);
        }
    }

    @Override
    public boolean isFinished() {
        return targetReached;
    }

    @Override
    public void end(boolean interrupted) {
        subsys.setSpeeds(new ChassisSpeeds(0,0,0), false);  
    }
}
