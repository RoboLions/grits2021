package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 *
 */
public class ManualRollIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final static XboxController manipulatorController = RobotContainer.manipulatorController;
    
    public ManualRollIntake(IntakeSubsystem intake) {
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
      }

    @Override
    public void initialize() {
    	
    }

    @Override
    public void execute() {
        // boolean start = manipulatorController.getStartButton(); // outtake
        // boolean back = manipulatorController.getBackButton(); // intake

        boolean mLeftTrigger = manipulatorController.getTriggerAxis(Hand.kLeft) > 0.25;
        boolean mRightTrigger = manipulatorController.getTriggerAxis(Hand.kRight) > 0.25;

        if(mRightTrigger) {
            intakeSubsystem.intakeBalls();
        } else if(mLeftTrigger) {
            intakeSubsystem.outtakeBalls();
        } else {
            intakeSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}