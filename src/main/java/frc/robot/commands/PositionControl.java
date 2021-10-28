/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//autonomous code for position control

public class PositionControl extends CommandBase {

    public WPI_TalonSRX colorWheelMotor = RobotMap.colorWheelMotor;
    // if uisng wheel 1/4 the size of color wheel

    public static int MEASURE_FOR_ROTATION = 4096 ;
    //variable for 1 rotatiion
    
    public PositionControl() {
        
    }

    public void initialize() {

    }

    public void execute() {
      //color sensing code
      String fms = RobotContainer.colorWheelSubsystem.getColorFromFMS();
      String current = RobotContainer.colorWheelSubsystem.detectSensorColor();

        if(fms == "blue") {
            if(current == "red") {
                RobotContainer.colorWheelSubsystem.stopMotor();
            }  else {
                RobotContainer.colorWheelSubsystem.moveMotorForward();
            } 
        } else if(fms == "red") {
            if(current == "blue") {
                RobotContainer.colorWheelSubsystem.stopMotor();
            } else {
                RobotContainer.colorWheelSubsystem.moveMotorForward();
            }
        } else if(fms == "green") {
            if(current == "yellow") {
                RobotContainer.colorWheelSubsystem.stopMotor();
            } else {
                RobotContainer.colorWheelSubsystem.moveMotorForward();
            }
        } else if(fms == "yellow") {
            if(current == "green") {
                RobotContainer.colorWheelSubsystem.stopMotor();
            } else {
                RobotContainer.colorWheelSubsystem.moveMotorForward();
            }
        }
    }
    

    public boolean isFinished() {
        return false;
    }

    protected void end() {

    }

    protected void interrupted() {
        end();   
    }  */
    
    /*
    if (fms = red && current == blue) {
            int MEASURE_FOR_ROTATION = 0 ;
        }
        else if (fms = red && current == yellow) {
            int MEASURE_FOR_ROTATION = 2048;
        }
        else if (fms = red && current == green) {
            int MEASURE_FOR_ROTATION = -2048;
        }
        else if (fms = red && current == red) {
            int MEASURE_FOR_ROTATION = 4096;
        }
        else if (fms = blue && current == red) {
            int MEASURE_FOR_ROTATION = 0 ;
        }
        else if (fms = blue && current == green) {
            int MEASURE_FOR_ROTATION = 2048;
        }
        else if (fms = blue && current == yellow) {
            int MEASURE_FOR_ROTATION = -2048;
        }
        else if (fms = blue && current == blue) {
            int MEASURE_FOR_ROTATION = 4096;
        } 
        else if (fms = green && current == yellow) {
            int MEASURE_FOR_ROTATION = 0 ;
        }
        else if (fms = green && current == red) {
            int MEASURE_FOR_ROTATION = 2048;
        }
        else if (fms = green && current == blue) {
            int MEASURE_FOR_ROTATION = -2048;
        }
        else if (fms = green && current == green) {
            int MEASURE_FOR_ROTATION = 4096;
        } 
        else if (fms = yellow && current == green) {
            int MEASURE_FOR_ROTATION = 0 ;
        }
        else if (fms = yellow && current == blue) {
            int MEASURE_FOR_ROTATION = 2048;
        }
        else if (fms = yellow && current == red) {
            int MEASURE_FOR_ROTATION = -2048;
        }
        else if (fms = yellow && current == yellow) {
            int MEASURE_FOR_ROTATION = 4096;
        } 
        else if (colorSensor.getColor.unknown()) {
            int MEASURE_FOR_ROTATION = 0 ;
        }*/     
//}   