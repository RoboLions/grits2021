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
public class AutoPath3Mirror extends SequentialCommandGroup {
  /**
   * Creates a far baseline
   */
  public AutoPath3Mirror(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoMove(driveSubsystem, 2.7), new StopNWait(driveSubsystem, 0.5),
        //move straight
        new AutoTurn(driveSubsystem, 90, 0.6), new StopNWait(driveSubsystem, 0.5),
        //turn right
        new AutoMove(driveSubsystem, 3.75), new StopNWait(driveSubsystem, 0.5),
        //move straight
        new AutoTurn(driveSubsystem, -90, 0.6), new StopNWait(driveSubsystem, 0.5),
        //position to dump
        new AutoMove(driveSubsystem, 0.25), new StopNWait(driveSubsystem, 0.5), 
        //move straight
        new Outtake(intakeSubsystem).withTimeout(1));  
        //lower arm and then dump into bottom port      
  }
}