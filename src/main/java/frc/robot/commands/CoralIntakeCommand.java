package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.RobotContainer;

public class CoralIntakeCommand extends Command {
    private CoralIntakeSubsystem subsys;
    private CommandXboxController controller;

    public CoralIntakeCommand(CoralIntakeSubsystem subsys) {
        this.subsys = subsys;
        this.controller = RobotContainer.getInstance().operatorController;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (controller != null && controller.getRightTriggerAxis() >= 0.5) {
            subsys.setPower(0.2);
        } else if(subsys.getMotorCurrent() > 30) {
            subsys.setPower(0);
            subsys.mode = CoralIntakeSubsystem.IntakeMode.STOP;
        } else { 
            subsys.setPower(subsys.mode.getPower());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
