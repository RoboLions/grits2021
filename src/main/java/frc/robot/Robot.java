/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonomousPaths.TestPath;
import frc.robot.commands.autonomous_paths.AutoMoveForward;
import frc.robot.commands.autonomous_paths.AutoPath0;
import frc.robot.commands.autonomous_paths.AutoPath1;
import frc.robot.commands.autonomous_paths.AutoPath2;
import frc.robot.lib.RoboLionsPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static RobotContainer m_robotContainer;

  private RoboLionsPID leftDrivetrainPID = m_robotContainer.driveSubsystem.leftForwardPID;
  private RoboLionsPID rightDrivetrainPID = m_robotContainer.driveSubsystem.rightForwardPID;

  private DriveSubsystem driveSubsystem = m_robotContainer.driveSubsystem;
  private IntakeSubsystem intakeSubsystem = m_robotContainer.intakeSubsystem;
  private ArmSubsystem armSubsystem = m_robotContainer.armSubsystem;

  private XboxController driverController = m_robotContainer.driverController;
  private WPI_TalonFX leftMotor = RobotMap.leftDriveMotor;
  private WPI_TalonFX rightMotor = RobotMap.rightDriveMotor;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.driveSubsystem.setModePercentVoltage();
    m_robotContainer.driveSubsystem.resetEncoders();
    m_robotContainer.driveSubsystem.ZeroYaw();
    m_chooser.setDefaultOption("Default Move Forward/Cross Line", new AutoMoveForward(driveSubsystem));
    m_chooser.addOption("Near: Empty into Lower", new AutoPath0(driveSubsystem, intakeSubsystem));
    m_chooser.addOption("Near: Empty into Lower, Back, 160 Turn", new AutoPath1(driveSubsystem, intakeSubsystem, armSubsystem));
    // m_chooser.addOption("Middle: Empty into Lower, Back, 130 Turn", new AutoPath2(driveSubsystem, intakeSubsystem, armSubsystem));
    m_chooser.addOption("Middle: Empty into Lower - 90 Degree Path", new TestPath(driveSubsystem, intakeSubsystem, armSubsystem));

    SmartDashboard.putData("Autonomous Chooser", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *\
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("ARM Pitch", -m_robotContainer.armSubsystem.getPitch());
    SmartDashboard.putNumber("Yaw", m_robotContainer.driveSubsystem.getYaw());
    // inverted pitch because of how everybot is built
    /*
     * SmartDashboard.putNumber("leftSpeed",
     * m_robotContainer.driveSubsystem.leftSpeed);
     * SmartDashboard.putNumber("rightSpeed", rightSpeed);
     * SmartDashboard.putNumber("Left Encoder V",
     * getLeftEncoderVelocityMetersPerSecond());
     * SmartDashboard.putNumber("Right Encoder V",
     * getRightEncoderVelocityMetersPerSecond());
     * SmartDashboard.putNumber("Left Motor Voltage",
     * leftMotor.getMotorOutputVoltage());
     * SmartDashboard.putNumber("Right Motor Voltage",
     * rightMotor.getMotorOutputVoltage());
     * SmartDashboard.putNumber("Distance Travelled", distanceTravelledinMeters());
     */
    SmartDashboard.putNumber("Left Encoder Counts", m_robotContainer.driveSubsystem.getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Counts", m_robotContainer.driveSubsystem.getRightEncoderPosition());
    SmartDashboard.putNumber("Left Dist Meters", m_robotContainer.driveSubsystem.leftDistanceTravelledInMeters());
    SmartDashboard.putNumber("Right Dist Meters", m_robotContainer.driveSubsystem.rightDistanceTravelledInMeters());
    SmartDashboard.putNumber("Temp Left F500", RobotMap.leftDriveMotor.getTemperature());
    SmartDashboard.putNumber("Temp Right F500", RobotMap.rightDriveMotor.getTemperature());
    SmartDashboard.putNumber("Left Motor Voltage", RobotMap.leftDriveMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Motor Voltage", RobotMap.rightDriveMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Climb Encoder Counts", m_robotContainer.climberSubsystem.getEncoderPosition());
  }

  @Override
  public void disabledPeriodic() {

  }
  
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
                // Autonomous PID
                leftDrivetrainPID.initialize2(
                  2, // Proportional Gain //2.925 ZN w FF 
                  20, // Integral Gain //42.12 ZN w FF
                  0.0, // Derivative Gain //0
                  0.0, // Cage Limit 0.3 //0
                  0.0, // Deadband //0
                  12,// MaxOutput Volts 0.25 //100 //12
                  false, //enableCage
                  false //enableDeadband
              );
      
              // Autonomous PID
              rightDrivetrainPID.initialize2(
                  2, // Proportional Gain //2.925 ZN w FF //2
                  20, // Integral Gain //42.12 ZN w FF //20
                  0.0, // Derivative Gain //0
                  0.0, // Cage Limit //0.3
                  0.0, // Deadband //0
                  12,// MaxOutput Volts 0.25 //100 //12
                  false, //enableCage
                  false //enableDeadband
              );
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.driveSubsystem.resetEncoders();
    m_robotContainer.driveSubsystem.ZeroYaw();

            // Rate Drive PID
            leftDrivetrainPID.initialize2(
              0, // Proportional Gain //2.925 ZN w FF 
              0, // Integral Gain //42.12 ZN w FF
              0.0, // Derivative Gain //0
              0.0, // Cage Limit 0.3 //0
              0.0, // Deadband //0
              12,// MaxOutput Volts 0.25 //100 //12
              false, //enableCage
              false //enableDeadband
          );
  
          // Rate Drive PID
          rightDrivetrainPID.initialize2(
              0, // Proportional Gain //2.925 ZN w FF //2
              0, // Integral Gain //42.12 ZN w FF //20
              0.0, // Derivative Gain //0
              0.0, // Cage Limit //0.3
              0.0, // Deadband //0
              12,// MaxOutput Volts 0.25 //100 //12
              false, //enableCage
              false //enableDeadband
          );
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // System.out.println(RobotContainer.driverController.getY(Hand.kLeft));

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
