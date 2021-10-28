package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class ManualMoveArm extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final XboxController manipulatorController = RobotContainer.manipulatorController;

    public static int wrist_motion_state = 0;

    public ManualMoveArm(ArmSubsystem arm) {
        armSubsystem = arm;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //System.out.println(-armSubsystem.getPitch());
        double armPower = manipulatorController.getY(Hand.kLeft);
        boolean x = manipulatorController.getXButton();
        // x button = ground
        boolean b = manipulatorController.getBButton();
        // b button = score

        SmartDashboard.putNumber("Pitch", armSubsystem.getPitch());

        switch(wrist_motion_state) {
            case 0:
                armSubsystem.setArmPower(armPower);
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }
                if(x) {
                    wrist_motion_state = 1;
                }
                if(b) {
                    wrist_motion_state = 2;
                }
                break;
            case 1:
            // ground
                armSubsystem.setArmToGround();
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }                
                break;
            case 2:
            // score
                armSubsystem.setArmToScore();
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }                
                break;
            default:
                wrist_motion_state = 0;
                break;
        }
        if(Math.abs(armPower) > 0.5 ) { 
            // wakes up the arm from PID control and back to joystick control
            wrist_motion_state = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}