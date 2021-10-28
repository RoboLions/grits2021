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
public class AutoPath9 extends SequentialCommandGroup {
    /**
     * Creates a trench(3 ball return).
     */
    public AutoPath9(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
      // Add your commands in the super() call, e.g.
      // super(new FooCommand(), new BarCommand());
      super(new AutoTurn(driveSubsystem, -27, 0.6), new StopNWait(driveSubsystem, 0.5),
      // turn left
          new AutoMove(driveSubsystem, 2.85), new StopNWait(driveSubsystem, 0.3),
          //go straight
          new Outtake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.3), 
          //outtake powercells
          new AutoTurn(driveSubsystem, 180, 0.6), new StopNWait(driveSubsystem, 0.5),
          //make a 180
          new AutoMove(driveSubsystem, 6.9), new StopNWait(driveSubsystem, 0.5),
          //move straight
          new AutoTurn(driveSubsystem, 45, 0.6), new StopNWait(driveSubsystem, 0.5),
          //turn right
          new AutoMove(driveSubsystem, 0.5), new StopNWait(driveSubsystem, 0.5),
          //move a little
          new AutoMoveArm(armSubsystem, Position.GROUND), new StopNWait(driveSubsystem, 0.5),
          //lower arm
          new Intake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.5),
          //
          new AutoMove(driveSubsystem, 1.5), new StopNWait(driveSubsystem, 0.5),
          //go straiti
          
          new AutoTurn(driveSubsystem, 90, 0.6), new StopNWait(driveSubsystem, 0.5),
          //
          new AutoMove(driveSubsystem, 1.5), new StopNWait(driveSubsystem, 0.5),
          //pick up balls
          new AutoTurn(driveSubsystem, 90, 0.6), new StopNWait(driveSubsystem, 0.5),
          //
          new AutoMove(driveSubsystem, 2.85), new StopNWait(driveSubsystem, 0.5));
          //start to make a u turn

  }
}
