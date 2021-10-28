/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous_paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMoveArm;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.StopNWait;
import frc.robot.commands.AutoMove.Mode;
import frc.robot.commands.AutoMoveArm.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath4Mirror extends SequentialCommandGroup {
  /**
   * Creates a new Far 2 cycles.
   */
  public AutoPath4Mirror(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoMove(driveSubsystem, 2.5), new StopNWait(driveSubsystem, 0.5),
        //go straight
        new AutoTurn(driveSubsystem, 90, 0.6), new StopNWait(driveSubsystem, 0.5),
        //turn right
        new AutoMove(driveSubsystem, 4.8), new StopNWait(driveSubsystem, 0.5),
        // move straight to the target zone
        new AutoTurn(driveSubsystem, -90, 0.6), new StopNWait(driveSubsystem, 0.5),
        //turn left
        new AutoMove(driveSubsystem, 0.25), new StopNWait(driveSubsystem, 0.5),
        //adjust position
        new Outtake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.3),
        //dump balls
        new AutoMove(driveSubsystem, -0.25), new StopNWait(driveSubsystem, 0.5),
        //move back
        new AutoTurn(driveSubsystem, -165, 0.6), new StopNWait(driveSubsystem, 0.5),
        //make u turn
        new AutoMove(driveSubsystem, 6.9), new StopNWait(driveSubsystem, 0.5),
        //move to rendevous
        new Intake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.3),
        //intake balls
        new AutoTurn(driveSubsystem, 105, 0.6),new StopNWait(driveSubsystem, 0.5),
        //turn right
        new AutoMove(driveSubsystem, 2.75), new StopNWait(driveSubsystem, 0.3),  
        //move straight
        new AutoTurn(driveSubsystem, -45, 0.6),new StopNWait(driveSubsystem, 0.5),
        //turn left and face the loading zone
        new AutoMove(driveSubsystem, 2.5), new StopNWait(driveSubsystem, 0.3),  
        //move straight
        new AutoTurn(driveSubsystem, 90, 0.6),new StopNWait(driveSubsystem, 0.5),
        //turn left and face the loading zone
        new AutoMove(driveSubsystem, 6.9), new StopNWait(driveSubsystem, 0.5),
        // move straight to the target zone
        new AutoTurn(driveSubsystem, -90, 0.6),new StopNWait(driveSubsystem, 0.5),
        //turn left and face the loading zone
        new AutoMove(driveSubsystem, 0.25), new StopNWait(driveSubsystem, 0.5),
        // move straight to the target zone
        new Outtake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.3)); 
        //dump into bottom port and go back    
  }
}

