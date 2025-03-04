package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.utils.TalonMotor;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final TalonMotor motor;
    public double requiredPower = 0;

    public AlgaeIntakeSubsystem() {
        motor = new TalonMotor(AlgaeIntakeConstants.motorConfig);
        setDefaultCommand(new AlgaeIntakeCommand(this));
    }

    public void setPower(double power) {
        motor.setDuty(power);
    }

    public void setRequiredPower(double power) {
        this.requiredPower = power;
    }
    public double getRequiredPower() {
        return this.requiredPower;
    }

    public void stop() {
        setPower(0);
    }

    public double getVelocity() {
        return motor.getCurrentVelocity();
    }

    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

}
