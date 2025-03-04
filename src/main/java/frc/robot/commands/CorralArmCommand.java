package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.Constants.AlgaeArmConstants;

public class CorralArmCommand extends Command {
    private CoralArmSubsystem subsys;
    private double angle = 0;
    private boolean stopped = false;


    public CorralArmCommand(CoralArmSubsystem subsys) {
        this.subsys = subsys;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        stopped = false;
        angle = -1;
    }

    @Override
    public void execute() {
        double a = subsys.requiredPosition.getAngle();
        if(a != angle) {
            stopped = false;
            angle = a;
            subsys.motorToPosition(angle);
        }
        double error = Math.abs(subsys.getCurrentAngle() - angle);
        if(stopped && error > AlgaeArmConstants.MAX_ERROR && !subsys.atLimit()) {
            subsys.motorToPosition(angle);
            stopped = false;
        } else if (!stopped && (error <= AlgaeArmConstants.MAX_ERROR || subsys.atLimit())) {
            subsys.stop();
            stopped = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
