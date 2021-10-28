/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousPaths;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoMoveArm;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.StopNWait;
import frc.robot.commands.AutoMoveArm.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
  /**
   * This auto path starts at the middle of the field and the goes straight forward,
   * turns 90, goes straight until in front of the ports,
   * turns -90 and then goes forward to empty balls into the lower port
   * 
   * MADE TO AVOID OTHER ROBOTS WHEN IN MIDDLE
   * 
   * FRONT ROBOT WHEEL ON THE INITIATION LINE
   */

  public TestPath(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // move straight
    super (new AutoMove(driveSubsystem, 2.5) //2.7
    , new StopNWait(driveSubsystem, 0.5),
      //turn right
      new AutoTurn(driveSubsystem, 90), new StopNWait(driveSubsystem, 0.5), //0.2
      //move to in front of the target zone
      new AutoMove(driveSubsystem, 1.621), new StopNWait(driveSubsystem, 0.5),
      //turn left
      new AutoTurn(driveSubsystem, -90), new StopNWait(driveSubsystem, 0.6),
      //shoot out balls
      new AutoMove(driveSubsystem, 0.1), new StopNWait(driveSubsystem, 0.3),
      
      new Outtake(intakeSubsystem).withTimeout(1.5));
      //edge closer
      /*new AutoMove(driveSubsystem, 0.5), new StopNWait(driveSubsystem, 0.5),
      //dump
      new Outtake(intakeSubsystem).withTimeout(1.5));
      */
  }
}
