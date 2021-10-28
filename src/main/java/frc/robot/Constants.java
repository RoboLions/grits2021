package frc.robot;

//import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int MANIPULATOR_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;
    public static int LEFT_AXIS = 0;
    public static int RIGHT_AXIS = 1;
  }

  public static final class ArmConstants {
    public static final int TIMEOUT_MS = 10;
    public static final double kP = 0.036; //0.06 // 0.036 ZN 
    public static final double kI = 0.1125;
    public static final double kD = 0.00288;

    public static final double GROUND_POSITION = 0;
    public static final double SCORE_POSITION = 50;//56;
  }

  public static final class ClimbConstants {
    public static final int TIMEOUT_MS = 10;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double COLOR_WHEEL_POSITION = 20;
    public static final double MAX_POSITION = 30;
    public static final double READY_POSITION = 40;
    public static final double DOWN_POSITION = 0;
  }

  public static final class ColorWheelConstants {
    public static final double WHEEL_POWER = 0.3;

    public static final Color RED = new Color(0.465, 0.3803, 0.1563);
    public static final Color YELLOW = new Color(0.32, 0.5463, 0.132);
    public static final Color GREEN = new Color(0.186, 0.52767, 0.3553);
    public static final Color BLUE = new Color(0.1403, 0.4453, 0.409);

    public static final double CONTROL_PANEL_CIRCUMFERENCE = 100.530964915; //pi*diameter
    public static final double SPINNER_CIRCUMFERENCE = 0.0; //placeholder 
    public static final double TOTAL_ROTATIONS = (CONTROL_PANEL_CIRCUMFERENCE/SPINNER_CIRCUMFERENCE)*4;
    public static final double ENCODER_TICKS_PR = 1440; //placeholder 
    public static final double TOTAL_ENCODER_TICKS = TOTAL_ROTATIONS * ENCODER_TICKS_PR;
    public static double COUNT; //individual encoder ticks
  }
  
  public static final class LimelightConstants {
    //modes for limelight led light
    public static double FORCE_OFF = 1;
    public static double FORCE_BLINK = 2;
    public static double FORCE_ON = 3;

    //modes for limelight camera 
    public static double VISION_PROCESSOR = 0;
    public static double DRIVER_CAMERA = 1;
    public static final double LLAIMING = 0.035;
    public static final double MOTORGAIN = 0.75; //0.8  //0.7; //0.6

    public static double powerPortHeightInches = 81.25; //9 feet 6.25 inches
    //TODO find values
    public static double limelightHeightOffFloor = 0; 
    public static double limelightAngle = 0;
  }
}