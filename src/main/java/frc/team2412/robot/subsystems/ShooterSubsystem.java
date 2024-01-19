package frc.team2412.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;

public class ShooterSubsystem extends SubsystemBase {
	// CONSTANTS
	// MOTOR SPEED VALUES
	public static final double SPEAKER_SHOOT_SPEED = 0.8;
	public static final double AMP_SHOOT_SPEED = 0.2;
	public static final double ANGLE_CHANGE_SPEED = 0.15;
	// MOTOR INSTANCE VARIBLES
	public ShooterSubsystem() {
		TalonFX SHOOTER_TOP_MOTOR = new TalonFX(Hardware.SHOOTER_TOP_MOTOR_ID);
		TalonFX SHOOTER_BOTTOM_MOTOR = new TalonFX(Hardware.SHOOTER_BOTTOM_MOTOR_ID);
		TalonFX SHOOTER_ANGLE_MOTOR = new TalonFX(Hardware.SHOOTER_ANGLE_MOTOR_ID);
	}
}
