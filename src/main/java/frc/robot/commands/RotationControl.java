/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.Constants.ColorWheelConstants;

public class RotationControl extends CommandBase {

  public static void resetEncoder() {
    ColorWheelConstants.COUNT = 0;
  }

  public static void CountingSpins() {
    //first reset the encoder to 0
    //then the encoder starts counting the number of ticks
    //when the encoder reaches totalEncoderTicks, the spinner will stop
    resetEncoder();

    if (ColorWheelConstants.COUNT<=(ColorWheelConstants.TOTAL_ENCODER_TICKS-100)) {
      //continue spinning motor
      RobotMap.colorWheelMotor.set(0.3);
    } else if (ColorWheelConstants.COUNT<=ColorWheelConstants.TOTAL_ENCODER_TICKS) {
      //slow the motor
      RobotMap.colorWheelMotor.set(0.2);
    }
    else {
      //stop the motor
      RobotMap.colorWheelMotor.set(0);
    }
  }
  

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

}
