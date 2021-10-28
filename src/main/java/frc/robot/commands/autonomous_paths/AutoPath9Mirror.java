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
import frc.robot.commands.AutoMove.Mode;
import frc.robot.commands.AutoMoveArm.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath9Mirror extends SequentialCommandGroup {
    /**
     * Creates a trench(3 ball return).
     */
    public AutoPath9Mirror(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
      // Add your commands in the super() call, e.g.
      // super(new FooCommand(), new BarCommand());
      super(new AutoTurn(driveSubsystem, 27, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
      //position towards to the bottom port
          new AutoMoveArm(armSubsystem, Position.GROUND), new Intake(intakeSubsystem, Mode.TIME,1), new AutoTurn(driveSubsystem, 180, 0.6),
          //lower arm then dump
          new AutoTurn(driveSubsystem, 180, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE,25, 0.6), new AutoTurn(driveSubsystem, -45, 0.6), 
          //make a 180 and then head foward
          new AutoTurn(driveSubsystem, -45, 0.6), new AutoMoveArm(armSubsystem, Position.GROUND), 
          //turn and then lower arm
          new Intake(intakeSubsystem, Mode.TIME,1), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
          //pick up balls
          new AutoTurn(driveSubsystem, 90, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 5, 0.6),
          //start to make a u turn
          new AutoTurn(driveSubsystem, 90, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 8.75, 0.6));
          //complete the u turn
    }
  }