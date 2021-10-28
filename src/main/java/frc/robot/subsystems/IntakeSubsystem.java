package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

    public static final double IN_POWER = -0.6; // TODO tune value to proper
    public static final double OUT_POWER = 0.6; // TODO tune value to proper
    public static final double STOP_POWER = 0.0;

    private static final WPI_TalonSRX intakeMotor = RobotMap.intakeMotor;

    public IntakeSubsystem() {
        //intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.set(0.0);
    }

    public void intakeBalls() {
        intakeMotor.set(IN_POWER);
    }

    public void outtakeBalls() {
        intakeMotor.set(OUT_POWER);
    }

    public void stop() {
        intakeMotor.set(STOP_POWER);
    }
}