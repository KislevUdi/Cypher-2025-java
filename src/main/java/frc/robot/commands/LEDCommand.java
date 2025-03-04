package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;

public class LEDCommand extends Command {
    private LedSubsystem subsys;
    private int[] color;

    public LEDCommand(LedSubsystem subsys, int[] color) {
        this.subsys = subsys;
        this.color = color;
        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        subsys.changeColor(color);
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
