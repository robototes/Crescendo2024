package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	public static final double INTAKE_IN_SPEED = 0.3;
	public static final double INTAKE_FAST_OUT_SPEED = -0.7;

	IntakeSubsystem() {
		CANSparkMax INTAKE_MOTOR_BACK = new CANSparkMax(INTAKE_MOTOR_BACK_ID, MotorType.kBrushless);
		CANSparkMax INTAKE_MOTOR_RIGHT = new CANSparkMax(INTAKE_MOTOR_RIGHT_ID, MotorType.kBrushless);
		CANSparkMax INTAKE_MOTOR_FRONT = new CANSparkMax(INTAKE_MOTOR_FRONT_ID, MotorType.kBrushless);
		CANSparkMax INTAKE_MOTOR_LEFT = new CANSparkMax(INTAKE_MOTOR_LEFT_ID, MotorType.kBrushless);

		CANSparkMax CONVEYOR_BELT_ONE = new CANSparkMax(CONVEYOR_BELT_ONE_ID, MotorType.kBrushless);
		CANSparkMax CONVEYOR_BELT_TWO = new CANSparkMax(CONVEYOR_BELT_TWO_ID, MotorType.kBrushless);
	}
}
