package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoMove.Mode;


/**
 *
 */
public class Outtake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private static final int DEFAULT_TIME = 1;
    
    public Outtake(IntakeSubsystem intake, Mode mode, int time) {
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
        //withTimeout(time);
      }

      public Outtake(IntakeSubsystem intake) {
          intakeSubsystem = intake;
          addRequirements(intakeSubsystem);
          //withTimeout(DEFAULT_TIME);
      }

    @Override
    public void initialize() {
    	
    }

    @Override
    public void execute() {
        intakeSubsystem.outtakeBalls();
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