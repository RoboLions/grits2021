/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;
import frc.robot.Constants.ColorWheelConstants;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class ColorWheelSubsystem extends SubsystemBase {
  public WPI_TalonSRX colorWheelMotor = RobotMap.colorWheelMotor;
  public final static ColorMatch m_colorMatcher = new ColorMatch();
  public static Color detectedColor;

  public ColorWheelSubsystem() {
    m_colorMatcher.addColorMatch(ColorWheelConstants.BLUE);
    m_colorMatcher.addColorMatch(ColorWheelConstants.GREEN);
    m_colorMatcher.addColorMatch(ColorWheelConstants.RED);
    m_colorMatcher.addColorMatch(ColorWheelConstants.YELLOW);
  }

  public static void SmartDashboard() {
    SmartDashboard.putNumber("Encoder: Rotation", ColorWheelSubsystem.getTicks());
  }
  
  public static double getTicks() {
    double count = RobotMap.colorWheelMotor.getSelectedSensorPosition();
    return count;
  }

  public String detectSensorColor() {
    //colorSensor.getColor(); 
    String colorString;
    Color detectedColor = RobotMap.m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == ColorWheelConstants.RED) {
      colorString = "Red";
    }
    else if (match.color == ColorWheelConstants.YELLOW) {
      colorString = "Yellow";
    }
    else if (match.color == ColorWheelConstants.GREEN) {
      colorString = "Green";
    }
    else if (match.color == ColorWheelConstants.BLUE) {
      colorString = "Blue";
    }
    else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    //shows red color value that is detected
    SmartDashboard.putNumber("Green", detectedColor.green);
    //shows green color value that is detected
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    //shows blue color value that is detected
    SmartDashboard.putNumber("Confidence", match.confidence);

    SmartDashboard.putString("Detected Color", colorString);
    //shows the name of the color that is detected

    return colorString;
  }

  public String getColorFromFMS() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {
        switch (gameData.charAt(0)) {
            case 'B' :
                return "Blue";
            case 'G' :
                return "Green";
            case 'R' :
                return "Red";
            case 'Y' :
                return "Yellow";
            default :
                return "Error";
        }
    } else {
        return "Error";
    }
}

public void moveMotorForward() {
  colorWheelMotor.set(ColorWheelConstants.WHEEL_POWER);
}

public void moveMotorReverse() {
  colorWheelMotor.set(-ColorWheelConstants.WHEEL_POWER);
}

public void stopMotor() {
  colorWheelMotor.set(0.0);
}

  /*public static final int color;
  public static final int red;

  public static void countColors() {
    color = getColorSensor();

    switch (color) {
      case 1:
        color = red;
        ColorCount++;
        break;
      case 2:
        color = blue;
        ColorCount++;
        break;
      case 3:
        color = yellow;
        ColorCount++;
        break;
      case 4:
        color = green;
        ColorCount++;
        break;
    }

    if (color == red) {
      ColorCount++;
    }
    else if (color == blue) {
      ColorCount++;
    }
    else if (color == green) {
      ColorCount++;
    }
    else if(color == yellow) {
      ColorCount++;
    }

}

}
*/