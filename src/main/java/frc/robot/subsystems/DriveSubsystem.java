package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsMotionProfile;
import frc.robot.lib.RoboLionsPID;

public class DriveSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // 2 * Math.PI; // one rotation per second
    private static final double IN_TO_M = .0254;
    
    /* Every Bot Variables */
    private static final int timeoutMs = 10;
    private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
    private static final double DIAMETER_INCHES = 5.0; // Flex wheels on Everybot
    
	private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    // This is for a output gear side motor encoder
	// private static final double TICKS_PER_METER = MOTOR_ENCODER_CODES_PER_REV / WHEEL_CIRCUMFERENCE;

    private static final double GEAR_RATIO = 12.75;
    //private static final double RIGHT_GEAR_RATIO = 10.71;
    // This is for an encoder mounted to the motor
    private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
    private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
    private static final double BOT_WHEEL_TO_WHEEL_DIAMETER = 0.49;//METERS

    public boolean state_flag_motion_profile = true;

    //90 degrees /360 = 2*PI*R 
    private static final double HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE = (BOT_WHEEL_TO_WHEEL_DIAMETER * Math.PI)/360.0;

    private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
    private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;
    private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
    private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;

    private static XboxController driverController = Robot.m_robotContainer.driverController;

    private final PigeonIMU imu = RobotMap.drive_imu;

    public RoboLionsPID leftForwardPID = new RoboLionsPID();
    public RoboLionsPID rightForwardPID = new RoboLionsPID();
	public RoboLionsPID headingPID = new RoboLionsPID();
    //public RoboLionsPID limelightPID = new RoboLionsPID();
    public RoboLionsPID positionPID = new RoboLionsPID();
    public RoboLionsMotionProfile positionMotionProfile = new RoboLionsMotionProfile();
    public RoboLionsMotionProfile headingMotionProfile = new RoboLionsMotionProfile();

    public double left_speed_cmd;
    public double right_speed_cmd;


    public DriveSubsystem() {
        ZeroYaw();
        resetEncoders();

        leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
        rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());

        leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftFrontMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

        leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftBackMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
        
        rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightFrontMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

        rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightBackMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

		leftFrontMotor.setNeutralMode(NeutralMode.Coast);
        rightFrontMotor.setNeutralMode(NeutralMode.Coast);
        leftBackMotor.setNeutralMode(NeutralMode.Coast);
        rightBackMotor.setNeutralMode(NeutralMode.Coast);
        
        leftFrontMotor.setInverted(false); //--> enable if we determine they are not even
        rightFrontMotor.setInverted(true);
        leftBackMotor.setInverted(false); 
        rightBackMotor.setInverted(true);

        // Rate Drive PID
        leftForwardPID.initialize2(
            0, // Proportional Gain //2.925 ZN w FF 
            0, // Integral Gain //42.12 ZN w FF
            0, // Derivative Gain //0
            0.0, // Cage Limit 0.3 //0
            0.0, // Deadband //0
            12,// MaxOutput Volts 0.25 //100 //12
            false, //enableCage
            false //enableDeadband
        );

        // Rate Drive PID
        rightForwardPID.initialize2(
            0, // Proportional Gain //2.925 ZN w FF //2
            0, // Integral Gain //42.12 ZN w FF //20
            0.0, // Derivative Gain //0
            0.0, // Cage Limit //0.3
            0.0, // Deadband //0
            12,// MaxOutput Volts 0.25 //100 //12
            false, //enableCage
            false //enableDeadband
        );

        // Position Command PID for Autonomous and 
        positionPID.initialize2(
            0, // Proportional Gain //1.35 //2
            0, // Integral Gain //5 //10
            0.0, // Derivative Gain //0
            0.0, // Cage Limit //0.3 //0.1 //0.2
            0.0, // Deadband //0
            2.5,// MaxOutput Meters/sec 0.25 //100 //1
            true, //enableCage
            false //enableDeadband
        );

        // Heading Command PID for Autonomous and 
        headingPID.initialize2(
            10, // Proportional Gain //15 // 7.5
            30.0, // Integral Gain // 10
            0.0, // Derivative Gain 
            20, // Cage Limit //0.3
            0.0, // Deadband
            360, // MaxOutput Degrees/sec 0.25 //100 //180
            true, //enableCage
            false //enableDeadband
        );
    }

    // feedforward calculation
    public double calculateNew(double velocity, double acceleration, double ks, double kv, double ka) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    /*****************************************************************************
    * 2/15/20 Use this for anything that has to deal with closed loop rate control
    ******************************************************************************/
    public void straightDrive(double leftSpeed, double rightSpeed) {
        left_speed_cmd = leftSpeed;
        right_speed_cmd = rightSpeed;
        
        //final double leftFeedforward = calculateNew(leftSpeed, 0, 0.75, 2.72, 0); // 0.3, 0.2
        //final double rightFeedforward = calculateNew(rightSpeed, 0, 0, 0, 0);

        double batteryVoltage = RobotController.getBatteryVoltage(); // getting battery voltage from PDP via the rio

        if (batteryVoltage < 1) {
            batteryVoltage = 1;
        }

        double leftOutput = leftForwardPID.execute(leftSpeed, getLeftBackEncoderVelocityMetersPerSecond());
        double rightOutput = rightForwardPID.execute(rightSpeed, getRightBackEncoderVelocityMetersPerSecond());
        
        double LVoltagePercentCommand = ((leftOutput) / batteryVoltage);
        double RVoltagePercentCommand = ((rightOutput) / batteryVoltage);

        if (LVoltagePercentCommand > 1.0) {
            LVoltagePercentCommand = 1.0;
        }
        else if (LVoltagePercentCommand < -1.0) {
            LVoltagePercentCommand = -1.0;
        }

        if (RVoltagePercentCommand > 1.0) {
            RVoltagePercentCommand = 1.0;
        }
        else if (RVoltagePercentCommand < -1.0) {
            RVoltagePercentCommand = -1.0;
        }

        //SmartDashboard.putNumber("Right Motor Command", RVoltagePercentCommand);
        //SmartDashboard.putNumber("Left Motor Command", LVoltagePercentCommand);
        
        leftBackMotor.set(LVoltagePercentCommand);
        //rightBackMotor.set(RVoltagePercentCommand);


        // SmartDashboard.putNumber("leftSpeed", leftSpeed);
        // SmartDashboard.putNumber("rightSpeed", rightSpeed);
        
        SmartDashboard.putNumber("Left Encoder V", getLeftBackEncoderVelocityMetersPerSecond());
        SmartDashboard.putNumber("Right Encoder V", getRightBackEncoderVelocityMetersPerSecond());
        
        //SmartDashboard.putNumber("Yaw Value", getYaw());
        //SmartDashboard.putNumber("Distance Travelled", distanceTravelledinMeters());
        //SmartDashboard.putNumber("Left Encoder Counts", getLeftEncoderPosition());
        //SmartDashboard.putNumber("Right Encoder Counts", getRightEncoderPosition());
        
        SmartDashboard.putNumber("Left Dist Meters", leftDistanceTravelledInMeters());
        SmartDashboard.putNumber("Right Dist Meters", rightDistanceTravelledInMeters());
        
        //System.out.println(getLeftEncoderVelocityMetersPerSecond() + "," + getRightEncoderVelocityMetersPerSecond());
        // System.out.println("Left Error: " + (leftSpeed-getLeftEncoderVelocityMetersPerSecond()) + "/ Right Error: " + (getRightEncoderVelocityMetersPerSecond()-rightSpeed));
        // System.out.println("Debug Out  " + rightOutput + " /// " + rightFeedforward + " /// " + JoystickDrive.throttle);
    }

    /*****************************************************************************
    * 2/15/20 Use this for anything that has to deal with closed loop rate control
    ******************************************************************************/
    public void driveWithRotation(double linearTravelSpeed, double rotateSpeed) {
        // input speed is meters per second, input rotation is bot rotation 
        // speed in meters per second
        // dev bot requires the output to be inverted, everybot needs it to NOT be inverted
        linearTravelSpeed = (1*linearTravelSpeed);
        rotateSpeed = (rotateSpeed);
        double leftSpeed = (linearTravelSpeed + rotateSpeed);
        double rightSpeed = (linearTravelSpeed - rotateSpeed);
        straightDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Speed Command", left_speed_cmd);
        SmartDashboard.putNumber("Right Speed Command", right_speed_cmd);
    }

    public void setModePercentVoltage() {
        leftFrontMotor.set(ControlMode.PercentOutput, 0);
        rightFrontMotor.set(ControlMode.PercentOutput, 0);
        leftBackMotor.set(ControlMode.PercentOutput, 0);
        rightBackMotor.set(ControlMode.PercentOutput, 0);
    }
    
    public double getYaw() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	return -ypr[0];
    }
    
    public double getPitch() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	return ypr[1];
    }
    
    public double getRoll() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	return ypr[2];
    }

    
    public double[] getRPH() {
    	double[] ypr = new double[3];
		imu.getYawPitchRoll(ypr);
		ypr[0] = -ypr[0];
    	return(ypr);
    }

    public void ZeroYaw() {
    	imu.setYaw(0, timeoutMs);
    	imu.setFusedHeading(0, timeoutMs);
    }

    public double distanceTravelledinTicks() {
		return (getLeftBackEncoderPosition() + getRightBackEncoderPosition()) / 2;
    }
    
	public double getLeftFrontEncoderPosition() {
		return leftFrontMotor.getSelectedSensorPosition();
	}

	public double getRightFrontEncoderPosition() {
		return rightFrontMotor.getSelectedSensorPosition();
    }
    
    public double getLeftBackEncoderPosition() {
		return leftBackMotor.getSelectedSensorPosition();
	}

	public double getRightBackEncoderPosition() {
		return rightBackMotor.getSelectedSensorPosition();
	}
	
	public double getLeftFrontEncoderVelocity() {
		return leftFrontMotor.getSelectedSensorVelocity();
	}

	public double getRightFrontEncoderVelocity() {
        return rightFrontMotor.getSelectedSensorVelocity();
    }
    
    public double getLeftBackEncoderVelocity() {
		return leftBackMotor.getSelectedSensorVelocity();
	}

	public double getRightBackEncoderVelocity() {
        return rightBackMotor.getSelectedSensorVelocity();
    }
    
    public double getLeftBackEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double leftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity()*10); // /10
        // since getQuadVelocity is in encoder ticks, we have to convert it to meters
        leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
        return (leftVelocityMPS);
    }
    
    public double getRightBackEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double rightVelocityMPS = (rightBackMotor.getSelectedSensorVelocity()*10); // /10
        // since getQuadVelocity is in encoder ticks, we have to convert it to meters
        //Need to have a negative for right velocity since the motors are reversed on the opposite side
        rightVelocityMPS = rightVelocityMPS * METERS_PER_TICKS;
        return (rightVelocityMPS);
    }

    public double getAverageEncoderVelocityMetersPerSecond() {
        double velocityMPS = (getRightBackEncoderVelocityMetersPerSecond()+getLeftBackEncoderVelocityMetersPerSecond())*0.5;
        return (velocityMPS);
    }

    public double leftDistanceTravelledInMeters() {
        double left_dist = getLeftBackEncoderPosition() * METERS_PER_TICKS;
        return left_dist;
    }

    public double rightDistanceTravelledInMeters() {
        double right_dist = getRightBackEncoderPosition() * METERS_PER_TICKS;
        return right_dist;
    }

	public double distanceTravelledinMeters() {
        // left distance is negative because the encoder value on the 
        // left is negative when dev bot is pushed forward 2/15/20
        // Code Tested on Dev Bot, Works on 2/15/20
        double distanceTravelled = (leftDistanceTravelledInMeters() + rightDistanceTravelledInMeters()) / 2;
		return distanceTravelled;
	}

	public void resetEncoders() {
		leftFrontMotor.setSelectedSensorPosition(0);
        rightFrontMotor.setSelectedSensorPosition(0);
        leftBackMotor.setSelectedSensorPosition(0);
		rightBackMotor.setSelectedSensorPosition(0);
    }
    
    // Advanced Drivetrain utilizing PID
    public void driveRoboLionsPID(double throttle, double rotate) {
        //  throttle is in meters per second, rotate is bot rotation speed in meters per second
        driveWithRotation(throttle, rotate);
    }

    // basic, no frills, no PID, no nothing drivetrain
    public static void drive(double throttle, double rotate) {
        leftFrontMotor.set(throttle + rotate);
        rightFrontMotor.set(throttle - rotate);
        leftBackMotor.set(throttle + rotate);
        rightBackMotor.set(throttle - rotate);
    }
    
    /*****************************************************************************
    * 2/15/20 Use this for anything that has to deal with closed loop rate control
    ******************************************************************************/
    public void autoDrive(double distance, double heading) { // distance is in meters, heading is in degrees
        // double left_speed; 
        // double right_speed;
        double start_dist = distanceTravelledinMeters();
        if(state_flag_motion_profile) {
            positionMotionProfile.init(
                        start_dist, //start position
                        distance, // target position
                        1, // max vel //1.5 // 1
                        1, // max accel //1 // 0.5
                        0.02, // execution period 
                        1 // deceleration //2 // 0.5
            );
            state_flag_motion_profile = false;
        }

        double position_profile_command = positionMotionProfile.execute();
        double feed_forward_rate = positionMotionProfile.velocity_feed_forward;

        double headingFeedback = getYaw(); // in degrees
        double headingCommand = heading; 
        //TODO Pls check if I was supposed to put heading as a parameter
        double headingError = headingPID.execute(headingCommand, headingFeedback);
        double headingErrorMeters = HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE * headingError;

        //System.out.println(headingCommand + "," + headingFeedback);

        double position_feedback = distanceTravelledinMeters();
        //SmartDashboard.putNumber("Auto Distance", position_feedback);
        // positionError is in meters per second
        double positionError = positionPID.execute(position_profile_command, position_feedback);
        double positionCmdOut = (positionError+feed_forward_rate);

        // System.out.println("Cmd: " + position_profile_command + " Fb: " + position_feedback + " Vel: " + getAverageEncoderVelocityMetersPerSecond());
        // System.out.println("Feedback: " + position_feedback);
        // System.out.println("PID Error: " + positionError);
        //System.out.println("\n");

        // left_speed = output;
        // right_speed = output;

        // straightDrive(left_speed, right_speed);
        // Refer to the rate drive control diagram
        // We modulate our speed of the bot to close out
        // the position error, making it eventually zero
        driveWithRotation(positionCmdOut, headingErrorMeters);
        //driveWithRotation(0.0, headingErrorMeters);
        //driveWithRotation(positionError, 0);
        // riveWithRotation(0.5, 0.0);
         //System.out.println("Pos " + position_feedback + " PE " + positionError);
        // System.out.println("TD " + distance + " // DT " + position_feedback);
    }

    public void stop() {
        drive(0, 0);
    }
}