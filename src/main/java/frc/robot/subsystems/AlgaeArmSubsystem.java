package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.commands.AlgaeArmCommand;
import frc.robot.utils.SparkMotor;

public class AlgaeArmSubsystem extends SubsystemBase {
    public enum ArmPosition {
        UP(0),
        TAKE(71),
        PROCESSOR(20),
        DROP_ALGAE_L2(20),
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

        static SendableChooser<ArmPosition> getChooser(AlgaeArmSubsystem subsystem) {
            SendableChooser<ArmPosition> chooser = new SendableChooser<>();
            chooser.setDefaultOption("Home", ArmPosition.UP);
            chooser.addOption("TAKE", ArmPosition.TAKE);
            chooser.addOption("L2", ArmPosition.DROP_ALGAE_L2);
            chooser.addOption("Processor", ArmPosition.PROCESSOR);
            chooser.addOption("Test", ArmPosition.TESING);
            chooser.onChange(subsystem::setPosition);
            return chooser;
        }        

    }

    public ArmPosition requiredPosition = ArmPosition.UP;

    private SparkMotor motor1;
    private SparkMotor motor2;
    private DigitalInput limit;
    private DutyCycleEncoder absoluteEncoder;
    private double absOffset;



    public AlgaeArmSubsystem() {
        super();
        // Configure motors
        motor1 = new SparkMotor(AlgaeArmConstants.motor1Config);
        motor2 = new SparkMotor(AlgaeArmConstants.motor2Config);

        limit = new DigitalInput(AlgaeArmConstants.LIMIT_PORT);
        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(AlgaeArmConstants.ABS_ENCODER_PORT));
        
        SmartDashboard.putData(this);
        SmartDashboard.putNumber("current angle in degree", getCurrentDegree());
        absOffset = AlgaeArmConstants.ABS_ENCODER_OFFSET;
        resetEncoder();
        SmartDashboard.putData("Alge Arm", this);
        setDefaultCommand(new AlgaeArmCommand(this));
    }

    public void toggleIntake() {
        if (requiredPosition == ArmPosition.TAKE) {
            requiredPosition = ArmPosition.UP;
            RobotContainer.getInstance().algaeIntakeSubsystem.setRequiredPower(0);;
        } else {
            requiredPosition = ArmPosition.TAKE;
            RobotContainer.getInstance().algaeIntakeSubsystem.setRequiredPower(Constants.SystemValues.intakeAlgaePower);
        }
    }

    public void resetEncoder() {
        motor1.getEncoder().setPosition(0);
        motor2.getEncoder().setPosition(0);
    }

    public boolean atLimit() {
        return !limit.get();
    }

    public void motorToPosition(double angle) {
        motor1.setPositionVoltage(angle);
        motor2.setPositionVoltage(angle);
    }

    public void stop() {
        motor1.set(0);
        motor2.set(0);
    }

    public double getCurrentDegree() {
        return motor1.getEncoder().getPosition();
    }

    public double getAbsDegree() {
        return (absoluteEncoder.get() - absOffset) *360;
    }

    public void setPosition(ArmPosition position) {
        requiredPosition = position;
    }


    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", this::getCurrentDegree, null);
        builder.addBooleanProperty("At limit", this::atLimit, null);
        builder.addDoubleProperty("abs", this::getAbsDegree, null);
        builder.addDoubleProperty("test", ArmPosition::getTestAngle, ArmPosition::setTestAngle);
        SmartDashboard.putData("Algae Arm Position", ArmPosition.getChooser(this));
    }

}
