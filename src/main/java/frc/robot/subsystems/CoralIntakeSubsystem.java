package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.utils.TalonMotor;;

public class CoralIntakeSubsystem extends SubsystemBase {

    public enum IntakeMode {
        INTAKE(0.5), OUTTAKE(-0.2), STOP(0), OUTTAKE2(-0.3), SPECIAL(-0.35);

        private double power;
        IntakeMode(double power) {
            this.power = power;
        }
        public double getPower() {
            return power;
        }
    }
    private final TalonMotor motor;
    public IntakeMode mode = IntakeMode.STOP;

    public CoralIntakeSubsystem() {
        motor = new TalonMotor(CoralIntakeConstants.motorConfig);
        setDefaultCommand(new CoralIntakeCommand(null));
    }

    public void toggle(IntakeMode mode) {
        if(this.mode == mode) {
            this.mode = IntakeMode.STOP;
        } else {
            this.mode = mode;
        }
    }

    public void setPower(double power) {
        motor.setDuty(power);
    }

    public void stop() {
        setPower(0);
    }

    public double getVelocity() {
        return motor.getCurrentVelocity();
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

}
