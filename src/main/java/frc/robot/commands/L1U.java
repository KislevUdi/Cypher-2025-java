package frc.robot.commands;

public class L1U extends Command {
    private swerveSubsystem subsys;
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
