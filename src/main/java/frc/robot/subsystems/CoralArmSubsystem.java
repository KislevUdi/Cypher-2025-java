package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralArmlConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.CorralArmCommand;
import frc.robot.utils.TalonMotor;

public class CoralArmSubsystem extends SubsystemBase {

    public enum ArmPosition {
        HOME(0),
        L2(54),
        L3(135),
        INTAKE(45.5),
        SPECIAL(73),
        TESING(0);

        private final double angle;
        public static double testingAngle = 0;

        ArmPosition(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            if(this == TESING) {
                return testingAngle;
            }
            return angle;
        }

        public static double getTestAngle()   {
            return testingAngle;
        }
        public static void setTestAngle(double angle) { 
            testingAngle = angle;
        }

        static SendableChooser<ArmPosition> getChooser(CoralArmSubsystem subsystem) {
            SendableChooser<ArmPosition> chooser = new SendableChooser<>();
            chooser.setDefaultOption("Home", ArmPosition.HOME);
            chooser.addOption("L2", ArmPosition.L2);
            chooser.addOption("L2", ArmPosition.L3);
            chooser.addOption("Intake", ArmPosition.INTAKE);
            chooser.addOption("Special", ArmPosition.SPECIAL);
            chooser.addOption("Test", ArmPosition.TESING);
            chooser.onChange(subsystem::setPosition);
            return chooser;
        }        

    }
    private final TalonMotor motor;
    private final DigitalInput limit;
    private final DutyCycleEncoder absoluteEncoder;
    private double offset;
    public ArmPosition requiredPosition = ArmPosition.HOME;

    public CoralArmSubsystem() {
        motor = new TalonMotor(CoralArmlConstants.motorConfig);
        limit = new DigitalInput(CoralArmlConstants.LIMIT_PORT);
        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(CoralArmlConstants.ABS_ENCODER_PORT));
        offset = CoralArmlConstants.ABS_ENCODER_OFFSET;
        resetEncoder();
        SmartDashboard.putData("Coral Arm", this);
        setDefaultCommand(new CorralArmCommand(this));
    }
    
    public void toggleIntake() {
        if (requiredPosition == ArmPosition.INTAKE) {
            requiredPosition = ArmPosition.HOME;
            RobotContainer.getInstance().coralIntakeSubsystem.mode = CoralIntakeSubsystem.IntakeMode.STOP;
        } else {
            requiredPosition = ArmPosition.INTAKE;
            RobotContainer.getInstance().coralIntakeSubsystem.mode = CoralIntakeSubsystem.IntakeMode.INTAKE;
        }
    }
    public void toggleL3() {
        if (requiredPosition == ArmPosition.L3) {
            requiredPosition = ArmPosition.HOME;
        } else {
            requiredPosition = ArmPosition.L3;
        }
    }
    public void toggleL2() {
        if (requiredPosition == ArmPosition.L2) {
            requiredPosition = ArmPosition.HOME;
        } else {
            requiredPosition = ArmPosition.L2;
        }
    }



    public void resetEncoder() {
        //double pose = absoluteEncoder.get() - offset;
        motor.setPosition(0);
    }

    public boolean atLimit() {
        return !limit.get();
    }

    public void motorToPosition(double angle) {
        motor.setMotionMagic(angle/360);
    }

    public void stop() {
        motor.setDuty(0);
    }

    public double getCurrentAngle() {
        return motor.getCurrentPosition() * 360;
    }
    public void setPosition(ArmPosition position) {
        requiredPosition = position;
    }

    public double getAbsAngle(){
        return absoluteEncoder.get() - offset;
    }


    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", this::getCurrentAngle, null);
        builder.addBooleanProperty("At limit", this::atLimit, null);
        builder.addDoubleProperty("test", ArmPosition::getTestAngle, ArmPosition::setTestAngle);
        SmartDashboard.putData("Coral Arm Position", ArmPosition.getChooser(this));
    }


}
