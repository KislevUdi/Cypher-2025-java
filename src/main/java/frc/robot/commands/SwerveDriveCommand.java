package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

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
            controller.getRightX(),
            true
        );
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}