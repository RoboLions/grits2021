package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.AutoMove.Mode;

/**
 *
 */
public class Intake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private static final int DEFAULT_TIME = 1;
    
    public Intake(IntakeSubsystem intake, Mode mode, int time) {
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
       // withTimeout(time);
      }

      public Intake(IntakeSubsystem intake) {
          intakeSubsystem = intake;
          addRequirements(intakeSubsystem);
        //  withTimeout(DEFAULT_TIME);
      }

    @Override
    public void initialize() {
    	
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeBalls();
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