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
public class AutoPath8 extends SequentialCommandGroup {
  /**
   * Creates a Trench (3 ball)
   */
  public AutoPath8(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoTurn(driveSubsystem, -45, 0.6), new StopNWait(driveSubsystem, 0.5),
        //turn left
        new AutoMove(driveSubsystem, 2.85), new StopNWait(driveSubsystem, 0.5),
        //go straight
        new Outtake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.5),
        //use outtake
        new AutoTurn(driveSubsystem, 180, 0.6), new StopNWait(driveSubsystem, 0.5),
        //iniate u turn
        new AutoMove(driveSubsystem, 5.9), new StopNWait(driveSubsystem, 0.5),
        //go straight
        new AutoTurn(driveSubsystem, 45, 0.6), new StopNWait(driveSubsystem, 0.5),
        //turn right
        new AutoMoveArm(armSubsystem, Position.GROUND), new StopNWait(driveSubsystem, 0.5),
        //head to trench
        new Intake(intakeSubsystem).withTimeout(1),new AutoMove(driveSubsystem, 5));
        //move while sucking balls
  }
}
