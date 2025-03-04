package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
    private AlgaeIntakeSubsystem subsys;
    private CommandXboxController controller;

    public AlgaeIntakeCommand(AlgaeIntakeSubsystem subsys) {
        this.subsys = subsys;
        this.controller = RobotContainer.getInstance().operatorController;
        addRequirements(subsys);
    }

    @Override
    public void execute() {
        if (controller != null && controller.getLeftTriggerAxis() >= 0.2) {
            subsys.setPower(0.2);
        } else if (subsys.getCurrent() >= 27) {
            subsys.setPower(0);
        } else {
            subsys.setPower(subsys.getRequiredPower());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
