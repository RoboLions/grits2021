/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command that does nothing but takes a specified amount of time to finish.  Useful for
 * CommandGroups.  Can also be subclassed to make a command with an internal timer.
 */
public class StopNWait extends CommandBase {
  protected Timer m_timer = new Timer();
  private final double m_duration;
  private final DriveSubsystem driveSubsystem;

  /**
   * Creates a new WaitCommand.  This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  public StopNWait(DriveSubsystem drivetrain, double seconds) {
    m_duration = seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    driveSubsystem = drivetrain;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.stop();
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
