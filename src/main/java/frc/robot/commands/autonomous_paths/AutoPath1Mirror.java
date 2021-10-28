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
public class AutoPath1Mirror extends SequentialCommandGroup {
  /**
   * Creates a new Trench baseline
   */
  public AutoPath1Mirror(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoMove(driveSubsystem,2.8), new StopNWait(driveSubsystem, 0.5),
        //move straight 
        new Outtake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.3),
        //move to target zone and lower arm
        new AutoMove(driveSubsystem, -0.5), new StopNWait(driveSubsystem, 0.3),
        //back up
        new AutoTurn(driveSubsystem, 180, 0.6), new StopNWait(driveSubsystem, 0.5),
        //dump and then turn around
        new AutoMove(driveSubsystem, 2.3), new StopNWait(driveSubsystem, 0.5),
        //go straight
        new AutoTurn(driveSubsystem, 80, 0.6), new StopNWait(driveSubsystem, 0.5),
        //turn right
        new AutoMove(driveSubsystem, 5.3), new StopNWait(driveSubsystem, 0.5),
        //move straight
        new AutoTurn(driveSubsystem, 50, 0.6), new StopNWait(driveSubsystem, 0.5),
        //enter the trench
       new AutoMove(driveSubsystem, 1), new StopNWait(driveSubsystem, 0.5),
        //continue drivng through the trench
        new AutoMoveArm(armSubsystem, Position.GROUND), new StopNWait(driveSubsystem, 0.3),  
        //move arm down
        new Intake(intakeSubsystem).withTimeout(1)); 
        //suck powercells
  }
}