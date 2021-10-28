package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
//import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

public class RobotMap {
    public static final int LEFT_DRIVE_PORT = 1;
    public static final int RIGHT_DRIVE_PORT = 2;

    public static final int INTAKE_PORT = 10;
    public static final int CLIMBER_PORT = 11;
    public static final int WINCH_PORT = 12;
    public static final int ARM_PORT = 13;
    public static final int ARM_IMU_PORT = 14;
    public static final int COLOR_WHEEL_MOTOR_PORT = 15; // placeholder

    public static I2C.Port i2cPort = I2C.Port.kOnboard;

    
    /************************************************************************************************************/

    public static WPI_TalonFX leftDriveMotor = new WPI_TalonFX(LEFT_DRIVE_PORT);
    public static WPI_TalonFX rightDriveMotor = new WPI_TalonFX(RIGHT_DRIVE_PORT);
    
    public static WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_PORT);
    public static WPI_TalonSRX climberMotor = new WPI_TalonSRX(CLIMBER_PORT);
    public static WPI_TalonSRX winchMotor = new WPI_TalonSRX(WINCH_PORT);
    public static WPI_TalonSRX armMotor = new WPI_TalonSRX(ARM_PORT);
    public static WPI_TalonSRX colorWheelMotor = new WPI_TalonSRX(COLOR_WHEEL_MOTOR_PORT);

    public static PigeonIMU arm_imu = new PigeonIMU(ARM_IMU_PORT);
    public static PigeonIMU drive_imu = new PigeonIMU(winchMotor);

    //public static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
}