package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoMoveArm extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private Position position;

    public static boolean end_me;

    public AutoMoveArm(ArmSubsystem arm, Position pos) {
        armSubsystem = arm;
        position = pos;
        end_me = false;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(position == Position.GROUND) {
            armSubsystem.setArmToGround();
            if(armSubsystem.armPID.deadband_active) {
                end_me = true;
            }
        } else if(position == Position.SCORE) {
            armSubsystem.setArmToScore();
            if(armSubsystem.armPID.deadband_active) {
                end_me = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return end_me;
    }

    public enum Position {
        GROUND, SCORE
    }
}