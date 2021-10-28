package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualMoveClimb extends CommandBase {
    public static final double TOWARDS_FRONT_POWER = 0.5; 
    public static final double TOWARDS_BACK_POWER = -1; 

    public static final double ACTUAL_MAX_ENCODER_COUNTS = 22700; //22550
    public static final double MIN_ENCODER_COUNTS = 0;

    private final ClimberSubsystem climberSubsystem;
    private final XboxController driverController = RobotContainer.driverController;
    private final static XboxController testerController = RobotContainer.testController;

    public static int climb_motion_state = 0;

    public static double climberEncoderCounts;

    public ManualMoveClimb(ClimberSubsystem climb) {
        climberSubsystem = climb;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climberEncoderCounts = climberSubsystem.getEncoderPosition();
        double climbPower = 0;
        
        boolean left_bumper = driverController.getBumper(Hand.kLeft);
        boolean right_bumper = driverController.getBumper(Hand.kRight);

        /*boolean bringClimbToGround = driverController.getBackButton();
        if (bringClimbToGround) {
            climberSubsystem.setClimbToGround();
        }*/

        boolean test_left_bumper = testerController.getBumper(Hand.kLeft);
        boolean test_right_bumper = testerController.getBumper(Hand.kRight);

        
        if(left_bumper) {
            climbPower = TOWARDS_FRONT_POWER; // moving inwards
        } else if(test_right_bumper) {
            climbPower = TOWARDS_BACK_POWER; // moving outwards
        }

        if(testerController.getBackButtonPressed()) {
            climberSubsystem.resetEncoder();
        }

        // Let down climber
        if(left_bumper && (climberEncoderCounts > MIN_ENCODER_COUNTS)) {
            climbPower = TOWARDS_FRONT_POWER; // moving inwards
        } 
        // Pull up climber
        else if(right_bumper && (climberEncoderCounts < ACTUAL_MAX_ENCODER_COUNTS)) {
            climbPower = TOWARDS_BACK_POWER; // moving outwards
        } 
        
        else if(!left_bumper && !right_bumper && !test_left_bumper && !test_right_bumper) {
            climbPower = 0; // not moving based on bumpers
        }
        
        

        climberSubsystem.setClimbPower(climbPower);

        /*

        boolean x = driverController.getXButton();
        // x = Color Wheel
        boolean b = driverController.getBButton();
        // b = Max (Tallest possible height at 45 in)
        boolean a = driverController.getAButton();
        // a = Ready (Move to exact height needed to latch on)
        boolean y = driverController.getYButton();
        // y = Ground/Zero Position

        switch(climb_motion_state) {
            case 0:
                climberSubsystem.setClimbPower(climbPower);

                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }
                if(x) {
                    climb_motion_state = 1;
                }
                if(b) {
                    climb_motion_state = 2;
                }
                if(a) {
                    climb_motion_state = 0; // should be 3 but we need button A for Slo-Mo so this is what we're doing
                }
                if(y) {
                    climb_motion_state = 4;
                }
                break;
            case 1:
            // Color Wheel
                climberSubsystem.setClimbToColorWheel();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }                
                break;
            case 2:
            // Max Position
                climberSubsystem.setClimbToMax();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }                
                break;
            case 3:
            // Ready to latch on
                climberSubsystem.setClimbToReady();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }
                break;
            case 4:
            // Down to Robot Ground/Base
                climberSubsystem.setClimbToGround();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }
                break;
            default:
                climb_motion_state = 0;
                break;
        }
        if(left_bumper || right_bumper) {
            climb_motion_state = 0;
        }
        */
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}