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
public class AutoPath2Mirror extends SequentialCommandGroup {
  /**
   * Creates a new middle baseline
   */
  public AutoPath2Mirror(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    super(new AutoTurn(driveSubsystem, 60, 0.6), new StopNWait(driveSubsystem, 0.5), 
        //turn
        new AutoMove(driveSubsystem, 2.8), new StopNWait(driveSubsystem, 0.3),
        //move straight and then move arm
        new Outtake(intakeSubsystem).withTimeout(1), new StopNWait(driveSubsystem, 0.5),
        //dump powercells
        new AutoTurn(driveSubsystem, 180, 0.6), new StopNWait(driveSubsystem, 0.3),
        // make a u turn
         new AutoMove(driveSubsystem, 2.8), new StopNWait(driveSubsystem, 0.3),
        //return to autoline
        new AutoTurn(driveSubsystem, -60, 0.6), new StopNWait(driveSubsystem, 0.3), 
        //make a left turm
        new AutoMove(driveSubsystem, 2.8), new StopNWait(driveSubsystem, 0.3),
        //head towards randevous point
        new AutoMoveArm(armSubsystem, Position.GROUND), new StopNWait(driveSubsystem, 0.3),
        //move arm
        new Intake(intakeSubsystem).withTimeout(1));
        //suck up POWERCELLS
    }
}
