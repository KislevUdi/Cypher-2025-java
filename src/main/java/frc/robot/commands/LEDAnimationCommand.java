package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;
import java.util.Arrays;

public class LEDAnimationCommand extends Command {
    private LedSubsystem subsys;
    private int[] startColor;
    private int[] endColor;
    private double rate;
    private boolean temp;
    private Timer time;

    public LEDAnimationCommand(LedSubsystem subsys, int[] startColor, int[] endColor, double rate) {
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

