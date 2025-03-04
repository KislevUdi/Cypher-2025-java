package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.Constants.AlgaeArmConstants;

public class AlgaeArmCommand extends Command {
    private AlgaeArmSubsystem subsys;
    private boolean stopped = false;
    private double angle = 0;

    public AlgaeArmCommand(AlgaeArmSubsystem subsys) {
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
        } else {
            double error = Math.abs(subsys.getCurrentDegree() - angle);
            if(stopped && error > AlgaeArmConstants.MAX_ERROR) {
                subsys.motorToPosition(angle);
                stopped = false;
            } else if (error <= AlgaeArmConstants.MAX_ERROR) {
                subsys.stop();
                stopped = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
