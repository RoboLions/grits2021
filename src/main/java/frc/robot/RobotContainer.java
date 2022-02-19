package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ManualMoveArm;
import frc.robot.commands.ManualMoveClimb;
import frc.robot.commands.ManualMoveWinch;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.DownMoveClimb;
import frc.robot.commands.ManualRollIntake;
import frc.robot.commands.UpMoveClimb;
import frc.robot.commands.AutonomousPaths.TestPath;
import frc.robot.commands.autonomous_paths.AutoPath1;
import frc.robot.commands.autonomous_paths.AutoPath2;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static WinchSubsystem winchSubsystem = new WinchSubsystem();
    public static ArmSubsystem armSubsystem = new ArmSubsystem();
    public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
    public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    //public static ColorWheelSubsystem colorWheelSubsystem = new ColorWheelSubsystem();

    // The driver's controller
    public static XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
   public static XboxController manipulatorController = new XboxController(OIConstants.MANIPULATOR_CONTROLLER_PORT);
    public static XboxController testController = new XboxController(OIConstants.TEST_CONTROLLER_PORT);

    // Auto Commands
    public static AutoPath1 autoPath1 = new AutoPath1(driveSubsystem, intakeSubsystem, armSubsystem);
    public static TestPath nearTrenchPath = new TestPath(driveSubsystem, intakeSubsystem, armSubsystem);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        driveSubsystem.setDefaultCommand(
            new JoystickDrive(driveSubsystem)
        );

        armSubsystem.setDefaultCommand(
            new ManualMoveArm(armSubsystem)
        );
        
        climberSubsystem.setDefaultCommand(
            new ManualMoveClimb(climberSubsystem)
        );

        winchSubsystem.setDefaultCommand(
            new ManualMoveWinch(winchSubsystem)
        );

        intakeSubsystem.setDefaultCommand(
            new ManualRollIntake(intakeSubsystem)
        );
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*** DRIVER CONTROLLER***/
        //Testing AutoMove and AutoTurn
        /*
        new JoystickButton(driverController, Button.kStart.value).whenPressed(
            new AutoMove(driveSubsystem, 10.0)
        );
        new JoystickButton(driverController, Button.kBack.value).whenPressed(
            new AutoTurn(driveSubsystem, 20)
        );
        */
    
        /*** MANIPULATOR CONTROLLER***/
       /* new JoystickButton(manipulatorController, Button.kY.value).whenPressed(
            new AutoPath2(driveSubsystem, intakeSubsystem, armSubsystem)
            //new TestPath(driveSubsystem, intakeSubsystem, armSubsystem)
        );
        */
    }
    
    public Command getAutonomousCommand() {
        return autoPath1;
    }
}
