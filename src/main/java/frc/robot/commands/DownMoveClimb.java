/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimberSubsystem;

public class DownMoveClimb extends CommandBase {
  /**
   * Creates a new SimpleMoveClimb.
   */
  public static final double TOWARDS_FRONT_POWER = 0.5; // TODO tune value to proper
  public static final double TOWARDS_BACK_POWER = -1; // TODO tune value to proper

  public static final int UPPER_MIN_ENCODER_COUNTS = 0; //TODO placeholder
  public static final int ACTUAL_MIN_ENCODER_COUNTS = 0; //TODO placeholder

  private final ClimberSubsystem climberSubsystem;
  private final XboxController driverController = RobotContainer.driverController;
  
  public DownMoveClimb(ClimberSubsystem climb) {
    climberSubsystem = climb;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climberSubsystem.setClimbPower(TOWARDS_BACK_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double climberEncoderCounts = climberSubsystem.getEncoderPosition();

    return (climberEncoderCounts < UPPER_MIN_ENCODER_COUNTS);
  }
}