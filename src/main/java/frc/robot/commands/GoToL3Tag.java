package frc.robot.commands;

import frc.robot.Constants.LimeLightConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GoToL3Tag extends GoToPose {
    private boolean left;
    private VisionSubsystem vision;

    public GoToL3Tag(boolean left, SwerveSubsystem swerve, VisionSubsystem vision) {
        super(null, swerve);
        this.left = left;
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