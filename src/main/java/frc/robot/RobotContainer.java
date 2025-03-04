package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.commands.GoToL3Tag;
import frc.robot.commands.LEDAnimationCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem.IntakeMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class RobotContainer implements Sendable {

    public boolean isRed = false;
    private static RobotContainer container = null;
    public static RobotContainer getInstance() {
        return container;
    }

    // Boolean flags
    private boolean led_bool_enable = true;
    private boolean led_bool_disable = true;

    // Controllers
    public final CommandXboxController driverController;
    public final CommandXboxController operatorController;

    // Subsystems
    public final SwerveSubsystem swerveSubsystem;
    public final CoralArmSubsystem coralArmSubsystem;
    public final CoralIntakeSubsystem coralIntakeSubsystem;
    public final LedSubsystem ledSubsystem;
    public final AlgaeIntakeSubsystem algaeIntakeSubsystem;
    public final AlgaeArmSubsystem algaeArmSubsystem;
    public final VisionSubsystem vision;

    // Commands
    private LEDCommand led_command_green;
    private LEDCommand led_command_blue;
    private LEDCommand led_command_red;
    private LEDCommand led_command_purple;
    private LEDCommand led_command_cyan;
    private LEDCommand led_command_yellow;
    
    private LEDAnimationCommand led_command_flash_blue;
    private LEDAnimationCommand led_command_flash_white;
    private LEDAnimationCommand led_command_flash_purple;
    private LEDAnimationCommand led_command_flash_cyan;
    
    // Command Groups
    private ParallelDeadlineGroup intakeAlgaeCommand;
    private ParallelDeadlineGroup intakeCoralCommand;
    private ParallelCommandGroup specialCoralIntakeCommand;

    public RobotContainer() {
        // Controllers
        driverController = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
        operatorController = new CommandXboxController(Constants.OIConstants.kOperatorControllerPort);

        // Subsystems
        swerveSubsystem = new SwerveSubsystem();
        coralArmSubsystem = new CoralArmSubsystem();
        ledSubsystem = new LedSubsystem();
        algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
        coralIntakeSubsystem = new CoralIntakeSubsystem();
        algaeArmSubsystem = new AlgaeArmSubsystem();
        vision = new VisionSubsystem(swerveSubsystem.getOdometer(), swerveSubsystem::getVelocity);
        configureButtonBindings();
        SmartDashboard.putData("Robot", this);
    }

/*     private void configureCommands() {
        // Commands
        specialIntakeCoralCommand = new CoralIntakeCommand(CoralIntakeSubsystem, Constants.SystemValues.specialCoralIntakePower);
        intakeAlgaeIntakeCommand = new AlgaeIntakeCommand(AlgaeIntakeSubsystem, Constants.SystemValues.intakeAlgaePower);
        outTakeAlgaeIntakeCommand = new AlgaeIntakeCommand(AlgaeIntakeSubsystem, Constants.SystemValues.outputAlgaePower);
        intakeCoralIntakeCommand = new CoralIntakeCommand(
            CoralIntakeSubsystem, Constants.SystemValues.intakeCoralPower
        );

        outputAlgaeIntakeCommand = new AlgaeIntakeCommand(AlgaeIntakeSubsystem, Constants.SystemValues.outputAlgaePower);
        outputCoralIntakeCommand = new CoralIntakeCommand(
            CoralIntakeSubsystem, Constants.SystemValues.outputCoralPower
        );
        outputCoralIntakeCommand2 = new CoralIntakeCommand(
            CoralIntakeSubsystem, -0.3
        );

        l3ArmCommand = new CoralArmCommand(CoralArmSubsystem, Constants.SystemValues.l3ArmAngle, true);
        l2ArmCommand = new CoralArmCommand(CoralArmSubsystem, Constants.SystemValues.l2ArmAngle, true);
        l0ArmCommand = new CoralArmCommand(CoralArmSubsystem, 0, true);

        stopIntakeCoralIntakeCommand = new CoralIntakeCommand(
            CoralIntakeSubsystem, 0
        );

        intakeCoralArmCommand = new CoralArmCommand(CoralArmSubsystem, Constants.SystemValues.intakeCoralArmAngle, true);
        specialIntakeCoralArmCommand = new CoralArmCommand(CoralArmSubsystem, Constants.SystemValues.specialCoralIntakeArmAngle, true);
        pickAlgaeArmCommand = new AlgaeArmCommand(AlgaeArmSubsystem, Constants.SystemValues.pickAlgaeArmAngle, true);
        outputAlgaeArmCommand = new AlgaeArmCommand(AlgaeArmSubsystem, Constants.SystemValues.ouputAlgaeArmAngle, true);

        led_command_green = new LEDCommand(led_subsys, new int[]{0, 200, 0});
        led_command_blue = new LEDCommand(led_subsys, new int[]{0, 0, 200});
        led_command_red = new LEDCommand(led_subsys, new int[]{200, 0, 0});
        led_command_purple = new LEDCommand(led_subsys, new int[]{200, 0, 200});
        led_command_cyan = new LEDCommand(led_subsys, new int[]{0, 200, 200});
        led_command_yellow = new LEDCommand(led_subsys, new int[]{240, 240, 0});

        led_command_flash_blue = new LEDAnimationCommand(led_subsys, new int[]{0, 0, 200}, new int[]{0, 0, 0}, 0.1);
        led_command_flash_white = new LEDAnimationCommand(led_subsys, new int[]{200, 200, 200}, new int[]{0, 0, 0}, 0.1);
        led_command_flash_purple = new LEDAnimationCommand(led_subsys, new int[]{200, 0, 200}, new int[]{0, 0, 0}, 0.1);
        led_command_flash_cyan = new LEDAnimationCommand(led_subsys, new int[]{0, 200, 200}, new int[]{0, 0, 0}, 0.1);

        // Default Commands
        deafultAlgaeIntakeCommand = new AlgaeIntakeCommand(
            AlgaeIntakeSubsystem, 0, true, operatorController
        );
        deafultCoralIntakeCommand = new CoralIntakeCommand(
            CoralIntakeSubsystem, 0, true
        );
        defaultCoralArmCommand = new CoralArmCommand(
            CoralArmSubsystem, 0, true
        );
        defaultAlgaeArmCommand = new AlgaeArmCommand(AlgaeArmSubsystem, 0, true);
        swerveCommand = new SwerveDriveCommand(
            swerveSubsystem, driverController
        );
        led_subsys.setDefaultCommand(led_command_yellow);

        // Set Default Commands
        swerveSubsystem.setDefaultCommand(swerveCommand);
        // AlgaeArmSubsystem.setDefaultCommand(defaultAlgaeArmCommand);
        CoralArmSubsystem.setDefaultCommand(defaultCoralArmCommand);
        AlgaeIntakeSubsystem.setDefaultCommand(deafultAlgaeIntakeCommand);
        CoralIntakeSubsystem.setDefaultCommand(deafultCoralIntakeCommand);

        // Command Groups
        intakeAlgaeCommand = new ParallelDeadlineGroup(
            intakeAlgaeIntakeCommand, pickAlgaeArmCommand, led_command_flash_blue
        );
        intakeCoralCommand = new ParallelDeadlineGroup(
            intakeCoralIntakeCommand, intakeCoralArmCommand, led_command_flash_purple
        );
        specialCoralIntakeCommand = new ParallelCommandGroup(
            specialIntakeCoralCommand, specialIntakeCoralArmCommand, led_command_flash_white
        );
    }
    */
    private void configureButtonBindings() {
        // Driver Controller
        driverController.b().onTrue(swerveSubsystem.runOnce(swerveSubsystem::zeroHeading));
        driverController.y().onTrue(swerveSubsystem.runOnce(swerveSubsystem::resetModulesAngle));
        driverController.a().toggleOnTrue(swerveSubsystem.runOnce(swerveSubsystem::toggelSlow));
        driverController.rightBumper().onTrue(new GoToL3Tag(false, swerveSubsystem, vision));
        driverController.leftBumper().onTrue(new GoToL3Tag(true, swerveSubsystem, vision));
        
        // Operator Controller
        operatorController.b().toggleOnTrue(algaeArmSubsystem.runOnce(algaeArmSubsystem::toggleIntake));
        operatorController.a().toggleOnTrue(coralArmSubsystem.runOnce(coralArmSubsystem::toggleIntake));
        operatorController.povLeft().toggleOnTrue(coralIntakeSubsystem.runOnce(()->coralIntakeSubsystem.mode = IntakeMode.STOP));
        operatorController.rightBumper().toggleOnTrue(coralArmSubsystem.runOnce(coralArmSubsystem::toggleL3));
        operatorController.leftBumper().toggleOnTrue(coralArmSubsystem.runOnce(coralArmSubsystem::toggleL2));
        
        operatorController.x().toggleOnTrue(coralIntakeSubsystem.runOnce(()->coralIntakeSubsystem.toggle(IntakeMode.OUTTAKE)));
        operatorController.y().onTrue(algaeIntakeSubsystem.runOnce(()->algaeIntakeSubsystem.requiredPower =Constants.SystemValues.intakeAlgaePower));
        operatorController.y().onFalse(algaeIntakeSubsystem.runOnce(()->algaeIntakeSubsystem.requiredPower = 0));
        
        // Driver
        driverController.povDown().toggleOnTrue(specialCoralIntakeCommand); // single driver
        driverController.povUp().whileTrue(new CoralIntakeCommand(CoralIntakeSubsystem, 0.2, true));
        driverController.povRight().toggleOnTrue(l0ArmCommand);
        
        operatorController.povDown().toggleOnTrue(specialCoralIntakeCommand);
        operatorController.povUp().toggleOnTrue(new CoralIntakeCommand(CoralIntakeSubsystem, 0.2, true));
        operatorController.povRight().toggleOnTrue(l0ArmCommand);
        operatorController.rightTrigger().toggleOnTrue(outputAlgiArmCommand);
        operatorController.leftTrigger().toggleOnTrue(defaultAlgiArmCommand);
    }
    
    public LEDCommand getYellowLEDCommand() {
        return led_command_yellow;
    }
    
    public LEDCommand getRedLEDCommand() {
        return led_command_red;
    }
    
    public AlgiArmSubsys getAlgiArmSubsys() {
        return algiArmSubsystem;
    }
    
    public Command getAutonomousCommand() {
        return new L1U(swerveSubsystem).andThen(outputCoralIntakeCommand2);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        // Implement if needed
    }
}
