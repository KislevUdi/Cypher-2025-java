package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import subsystem.SwerveSubsystem;
import subsystem.Limelight;
import constants.LimeLightConstants;

public class GoToL3Tag extends GoToPose {
    private boolean left;
    private SwerveSubsystem swerve;
    private Limelight vision;

    public GoToL3Tag(boolean left, SwerveSubsystem swerve, Limelight vision,
                  CommandXboxController controller) {
        super(null, swerve, controller);
        this.left = left;
        this.swerve = swerve;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        int tid = vision.getTagId();
        SmartDashboard.putNumber("Go To Tag Id", tid);
        if (tid > 0) {
            if (left) {
                pose = LimeLightConstants.getLeftL3Position(tid);
            } else {
                pose = LimeLightConstants.getRightL3Position(tid);
            }
        } else {
            pose = null;
        }
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

// GoToPose.java
package commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import subsystem.SwerveSubsystem;
import constants.LimeLightConstants;

public class GoToPose extends Command {
    protected static double driveKp = 0.4;
    protected static double omegaKp = 0.1;

    protected SwerveSubsystem subsys;
    protected Pose2d pose;
    protected boolean targetReached;
    protected CommandXboxController controller;

    public GoToPose(Pose2d pose, SwerveSubsystem subsys, CommandXboxController controller) {
        this.subsys = subsys;
        this.pose = pose;
        this.controller = controller;
        this.targetReached = false;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        targetReached = (pose == null);
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
            Pose2d error = pose.relativeTo(currentPose);
            double x = error.getTranslation().getX();
            double y = error.getTranslation().getY();
            double deg = error.getRotation().getDegrees();
            targetReached = Math.abs(x) < 0.02 && Math.abs(y) < 0.02 && Math.abs(deg) < 3;
            subsys.setSpeeds(new ChassisSpeeds(x * driveKp, y * driveKp, -deg * omegaKp), false);
        }
    }

    @Override
    public boolean isFinished() {
        return targetReached;
    }

    @Override
    public void end(boolean interrupted) {
        subsys.drive(0, 0, 0);
    }
}

// L1U.java
package commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import subsystem.SwerveSubsystem;

public class L1U extends Command {
    private SwerveSubsystem subsys;
    private int count;
    private Rotation2d rotation;
    private double x;
    private double y;

    public L1U(SwerveSubsystem subsys) {
        this.subsys = subsys;
        this.count = 0;
        this.rotation = subsys.getRotation2d();
        this.x = 0;
        this.y = 0;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        rotation = subsys.getRotation2d();
        x = rotation.getCos() * 0.6;
        y = rotation.getSin() * 0.6;
        subsys.drive(-x, -y, 0);
        count++;
    }

    @Override
    public void end(boolean interrupted) {
        subsys.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return count > 300;
    }
}

// LEDAnimationCommand.java
package commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import subsystem.LEDSubsys;
import constants.LED;

import java.util.Arrays;
import java.util.List;

public class LEDAnimationCommand extends Command {
    private LEDSubsys subsys;
    private int[] startColor;
    private int[] endColor;
    private double rate;
    private boolean temp;
    private Timer time;

    public LEDAnimationCommand(LEDSubsys subsys, int[] startColor, int[] endColor, double rate) {
        this.subsys = subsys;
        this.startColor = Arrays.copyOf(startColor, startColor.length);
        this.endColor = endColor;
        this.rate = rate;
        this.temp = true;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        subsys.changeColor(startColor);
        time = new Timer();
        time.start();
        super.initialize();
    }

    @Override
    public void execute() {
        if (time.get() >= rate) {
            temp = !temp;
            time.restart();
            if (temp) {
                subsys.changeColor(startColor);
            } else {
                subsys.changeColor(endColor);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

// LEDCommand.java
package commands;

import edu.wpi.first.wpilibj2.command.Command;
import subsystem.LEDSubsys;

public class LEDCommand extends Command {
    private LEDSubsys subsys;
    private int[] color;
    private boolean run;

    public LEDCommand(LEDSubsys subsys, int[] color) {
        this.subsys = subsys;
        this.color = color;
        this.run = false;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        subsys.changeColor(color);
        run = true;
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return run;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

// SwerveDriveCommand.java
package commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import subsystem.SwerveSubsystem;

public class SwerveDriveCommand extends Command {
    private SwerveSubsystem subsys;
    private CommandXboxController controller;

    public SwerveDriveCommand(SwerveSubsystem subsys, CommandXboxController controller) {
        this.subsys = subsys;
        this.controller = controller;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        subsys.drive(
            -controller.getLeftY(),
            controller.getLeftX(),
            controller.getRightX()
        );
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}