package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.Robot;
import frc.team2412.robot.util.SparkPIDWidget;
import java.util.Map;

public class LauncherSubsystem extends SubsystemBase {
	// CONSTANTS

	// HARDWARE
	private static final double PIVOT_GEARING_RATIO = 1.0 / 180.0;
	// ANGLE VALUES
	public static final int AMP_AIM_ANGLE = 335;
	public static final int SUBWOOFER_AIM_ANGLE = 298;
	public static final int PODIUM_AIM_ANGLE = 39;
	public static final int TRAP_AIM_ANGLE = 290;
	// MOTOR VALUES
	// max Free Speed: 6784 RPM
	private static final int MAX_FREE_SPEED_RPM = 6784;
	public static final double ANGLE_TOLERANCE = 5;
	public static final double RPM_TOLERANCE = 200;
	// RPM
	public static final int SPEAKER_SHOOT_SPEED_RPM = 3850;
	public static final int TRAP_SHOOT_SPEED_RPM = 4000;
	public static final double ANGLE_MAX_SPEED = 1.0;
	// 3392 RPM = 50% Speed
	// 1356 RPM = 20% Speed
	// 1017 RPM = 15% Speed

	// HARDWARE
	private final CANSparkFlex launcherTopMotor;
	private final CANSparkFlex launcherBottomMotor;
	private final CANSparkFlex launcherAngleOneMotor;
	private final CANSparkFlex launcherAngleTwoMotor;
	private final RelativeEncoder launcherTopEncoder;
	private final RelativeEncoder launcherBottomEncoder;
	private final SparkAbsoluteEncoder launcherAngleEncoder;
	private final SparkPIDController launcherAngleOnePIDController;

	// arm FF values:
	// Ks: 0.40434
	// Kv: 0.096771
	// Ka: 0.0056403

	private final SparkPIDController launcherTopPIDController;
	private final SparkPIDController launcherBottomPIDController;
	private final SimpleMotorFeedforward launcherTopFeedforward =
			new SimpleMotorFeedforward(0, 0.11335, 0.048325);
	private final SimpleMotorFeedforward launcherBottomFeedforward =
			new SimpleMotorFeedforward(0, 0.11238, 0.048209);

	private double rpmSetpoint;
	private double angleSetpoint;

	private GenericEntry setLauncherSpeedEntry;

	private GenericEntry launcherAngleEntry;

	private GenericEntry launcherSpeedEntry;

	private GenericEntry launcherAngleSpeedEntry;

	private GenericEntry launcherIsAtSpeed;

	// Constructors
	public LauncherSubsystem() {

		// MOTOR INSTANCE VARIBLES
		// motors
		launcherTopMotor = new CANSparkFlex(Hardware.LAUNCHER_TOP_MOTOR_ID, MotorType.kBrushless);
		launcherBottomMotor = new CANSparkFlex(Hardware.LAUNCHER_BOTTOM_MOTOR_ID, MotorType.kBrushless);
		launcherAngleOneMotor =
				new CANSparkFlex(Hardware.LAUNCHER_PIVOT_ONE_MOTOR_ID, MotorType.kBrushless);
		launcherAngleTwoMotor =
				new CANSparkFlex(Hardware.LAUNCHER_PIVOT_TWO_MOTOR_ID, MotorType.kBrushless);
		// encoders
		launcherTopEncoder = launcherTopMotor.getEncoder();
		launcherBottomEncoder = launcherBottomMotor.getEncoder();
		launcherAngleEncoder = launcherAngleOneMotor.getAbsoluteEncoder(Type.kDutyCycle);

		// PID controllers
		// Create launcherTopPIDController and launcherTopMotor]
		launcherTopPIDController = launcherTopMotor.getPIDController();
		launcherTopPIDController.setFeedbackDevice(launcherTopEncoder);
		launcherBottomPIDController = launcherBottomMotor.getPIDController();
		launcherBottomPIDController.setFeedbackDevice(launcherBottomEncoder);
		launcherAngleOnePIDController = launcherAngleOneMotor.getPIDController();
		launcherAngleOnePIDController.setFeedbackDevice(launcherAngleEncoder);
		// launcherAngleTwoPIDController = launcherAngleTwoMotor.getPIDController();
		// launcherAngleTwoPIDController.setFeedbackDevice(launcherAngleEncoder);

		configMotors();
		initShuffleboard();
	}

	public void configMotors() {
		launcherTopMotor.restoreFactoryDefaults();
		launcherBottomMotor.restoreFactoryDefaults();
		launcherAngleOneMotor.restoreFactoryDefaults();
		launcherAngleTwoMotor.restoreFactoryDefaults();
		// idle mode (wow)
		launcherTopMotor.setIdleMode(IdleMode.kCoast);
		launcherBottomMotor.setIdleMode(IdleMode.kCoast);
		launcherAngleOneMotor.setIdleMode(IdleMode.kBrake);
		launcherAngleTwoMotor.setIdleMode(IdleMode.kBrake);
		// inveritng the bottom motor lmao
		launcherTopMotor.setInverted(true);
		// launcherAngleTwoMotor.setInverted(true);

		// current limit
		launcherTopMotor.setSmartCurrentLimit(40);
		launcherBottomMotor.setSmartCurrentLimit(40);
		launcherAngleOneMotor.setSmartCurrentLimit(60);
		launcherAngleTwoMotor.setSmartCurrentLimit(60);

		launcherAngleOneMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, 0.95f);
		launcherAngleOneMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 0.71f);
		launcherAngleOneMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
		launcherAngleOneMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

		launcherAngleTwoMotor.follow(launcherAngleOneMotor, true);

		// PID
		launcherAngleOnePIDController.setP(5.421);
		launcherAngleOnePIDController.setI(0);
		launcherAngleOnePIDController.setD(0.066248);
		launcherAngleOnePIDController.setOutputRange(-ANGLE_MAX_SPEED, ANGLE_MAX_SPEED);

		launcherTopPIDController.setP(7.7633E-05);
		launcherTopPIDController.setI(0);
		launcherTopPIDController.setD(0);

		launcherBottomPIDController.setP(0.00011722);
		launcherBottomPIDController.setI(0);
		launcherBottomPIDController.setD(0);

		launcherAngleOneMotor.getEncoder().setPosition(launcherAngleEncoder.getPosition());
		launcherAngleOneMotor.getEncoder().setPositionConversionFactor(PIVOT_GEARING_RATIO);
		launcherAngleTwoMotor.getEncoder().setPosition(launcherAngleEncoder.getPosition());
		launcherAngleTwoMotor.getEncoder().setPositionConversionFactor(PIVOT_GEARING_RATIO);

		launcherTopMotor.burnFlash();
		launcherBottomMotor.burnFlash();
		launcherAngleOneMotor.burnFlash();
		launcherAngleTwoMotor.burnFlash();
	}
	// stop launcher motors method
	public void stopLauncher() {
		launcherTopMotor.disable();
		launcherBottomMotor.disable();
	}

	// uses the value from the entry
	public void launch() {
		rpmSetpoint = setLauncherSpeedEntry.getDouble(SPEAKER_SHOOT_SPEED_RPM);
		launch(rpmSetpoint);
	}
	// used for presets
	public void launch(double speed) {
		rpmSetpoint = speed;
		launcherTopPIDController.setReference(
				rpmSetpoint, ControlType.kVelocity, 0, launcherTopFeedforward.calculate(speed));
		launcherBottomPIDController.setReference(
				rpmSetpoint, ControlType.kVelocity, 0, launcherBottomFeedforward.calculate(speed));
	}

	public void ampLaunch(double speed) {
		launcherTopPIDController.setReference(
				-speed, ControlType.kVelocity, 0, launcherTopFeedforward.calculate(-speed));
		launcherBottomMotor.disable();
	}

	public double getLauncherSpeed() {
		return launcherTopEncoder.getVelocity();
	}
	// returns the degrees of the angle of the launcher
	public double getAngle() {
		// get position returns a double in the form of rotations
		return Units.rotationsToDegrees(launcherAngleEncoder.getPosition());
	}

	public void setAngle(double launcherAngle) {
		angleSetpoint = launcherAngle;
		launcherAngleOnePIDController.setReference(
				Units.degreesToRotations(angleSetpoint), ControlType.kPosition);
		// launcherAngleTwoPIDController.setReference(
		//		Units.degreesToRotations(angleSetpoint), ControlType.kPosition);
	}

	public boolean isAtAngle(double tolerance) {
		return MathUtil.isNear(angleSetpoint, launcherAngleEncoder.getPosition(), tolerance);
	}

	public boolean isAtAngle() {
		return isAtAngle(ANGLE_TOLERANCE);
	}

	public boolean isAtSpeed(double tolerance) {
		return MathUtil.isNear(rpmSetpoint, launcherTopEncoder.getVelocity(), tolerance);
	}

	public boolean isAtSpeed() {
		return isAtSpeed(RPM_TOLERANCE);
	}

	public double getAngleSpeed() {
		return launcherAngleEncoder.getVelocity();
	}

	public void setAngleSpeed(double Speed) {
		// launcherAngleOnePIDController.setReference(Speed, ControlType.kVelocity);
		// launcherAngleTwoPIDController.setReference(Speed, ControlType.kVelocity);
		launcherAngleOneMotor.set(Speed);
	}

	private void initShuffleboard() {
		if (Robot.isDebugMode()) {
			Shuffleboard.getTab("Launcher")
					.add(new SparkPIDWidget(launcherAngleOnePIDController, "launcherAnglePID"))
					.withPosition(2, 0);
			// Shuffleboard.getTab("Launcher")
			//		.add(new SparkPIDWidget(launcherAngleTwoPIDController, "launcherAngleTwoPIDController"));
			Shuffleboard.getTab("Launcher")
					.add(new SparkPIDWidget(launcherTopPIDController, "launcherTopPID"))
					.withPosition(0, 0);
			Shuffleboard.getTab("Launcher")
					.add(new SparkPIDWidget(launcherBottomPIDController, "launcherBottomPID"))
					.withPosition(1, 0);

			Shuffleboard.getTab("Launcher")
					.addDouble("Bottom FlyWheel Temp", () -> launcherBottomMotor.getMotorTemperature());
			Shuffleboard.getTab("Launcher")
					.addDouble("Top FlyWheel Temp", () -> launcherTopMotor.getMotorTemperature());
		}

		launcherIsAtSpeed =
				Shuffleboard.getTab("Launcher")
						.add("flywheels at target speed", false)
						.withSize(1, 1)
						.withWidget(BuiltInWidgets.kBooleanBox)
						.withPosition(0, 2)
						.getEntry();
		launcherAngleSpeedEntry =
				Shuffleboard.getTab("Launcher")
						.add("Launcher angle Speed", 0)
						.withSize(3, 1)
						.withWidget(BuiltInWidgets.kTextView)
						.withPosition(5, 2)
						.getEntry();
		launcherSpeedEntry =
				Shuffleboard.getTab("Launcher")
						.add("Launcher Speed", 0)
						.withSize(3, 1)
						.withWidget(BuiltInWidgets.kTextView)
						.withPosition(5, 1)
						.getEntry();
		launcherAngleEntry =
				Shuffleboard.getTab("Launcher")
						.add("Launcher angle", 0)
						.withSize(3, 1)
						.withWidget(BuiltInWidgets.kTextView)
						.withPosition(5, 3)
						.getEntry();
		setLauncherSpeedEntry =
				Shuffleboard.getTab("Launcher")
						.add("Launcher Speed Setpoint", 0)
						.withSize(3, 1)
						.withWidget(BuiltInWidgets.kNumberSlider)
						.withProperties(Map.of("Min", -MAX_FREE_SPEED_RPM, "Max", MAX_FREE_SPEED_RPM))
						.withPosition(5, 0)
						.getEntry();
	}

	@Override
	public void periodic() {
		launcherAngleEntry.setDouble(getAngle());
		launcherSpeedEntry.setDouble(getLauncherSpeed());
		launcherAngleSpeedEntry.setDouble(getAngleSpeed());
		launcherIsAtSpeed.setBoolean(isAtSpeed());
	}

	private SysIdRoutine getArmSysIdRoutine() {
		return new SysIdRoutine(
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						(Measure<Voltage> volts) -> {
							launcherAngleOneMotor.setVoltage(volts.in(BaseUnits.Voltage));
							launcherAngleTwoMotor.setVoltage(volts.in(BaseUnits.Voltage));
						},
						(SysIdRoutineLog log) -> {
							log.motor("angle-one")
									.voltage(
											BaseUnits.Voltage.of(
													launcherAngleOneMotor.getAppliedOutput()
															* RobotController.getBatteryVoltage()))
									.angularPosition(
											edu.wpi.first.units.Units.Rotations.of(
													launcherAngleOneMotor.getEncoder().getPosition()))
									.angularVelocity(
											edu.wpi.first.units.Units.Rotations.per(edu.wpi.first.units.Units.Minute)
													.of(launcherAngleOneMotor.getEncoder().getVelocity()));
							log.motor("angle-two")
									.voltage(
											BaseUnits.Voltage.of(
													launcherAngleTwoMotor.getAppliedOutput()
															* RobotController.getBatteryVoltage()))
									.angularPosition(
											edu.wpi.first.units.Units.Rotations.of(
													launcherAngleTwoMotor.getEncoder().getPosition()))
									.angularVelocity(
											edu.wpi.first.units.Units.Rotations.per(edu.wpi.first.units.Units.Minute)
													.of(launcherAngleTwoMotor.getEncoder().getVelocity()));
						},
						this));
	}

	private SysIdRoutine getFlywheelSysIdRoutine() {
		return new SysIdRoutine(
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						(Measure<Voltage> volts) -> {
							launcherTopMotor.setVoltage(volts.in(BaseUnits.Voltage));
							launcherBottomMotor.setVoltage(volts.in(BaseUnits.Voltage));
						},
						(SysIdRoutineLog log) -> {
							log.motor("top-flywheel")
									.voltage(
											BaseUnits.Voltage.of(
													launcherTopMotor.getAppliedOutput()
															* RobotController.getBatteryVoltage()))
									.angularPosition(
											edu.wpi.first.units.Units.Rotations.of(launcherTopEncoder.getPosition()))
									.angularVelocity(
											edu.wpi.first.units.Units.Rotations.per(edu.wpi.first.units.Units.Minute)
													.of(launcherTopEncoder.getVelocity()));
							log.motor("bottom-flywheel")
									.voltage(
											BaseUnits.Voltage.of(
													launcherBottomMotor.getAppliedOutput()
															* RobotController.getBatteryVoltage()))
									.angularPosition(
											edu.wpi.first.units.Units.Rotations.of(launcherBottomEncoder.getPosition()))
									.angularVelocity(
											edu.wpi.first.units.Units.Rotations.per(edu.wpi.first.units.Units.Minute)
													.of(launcherBottomEncoder.getVelocity()));
						},
						this));
	}

	public Command armSysIdQuasistatic(SysIdRoutine.Direction direction) {
		return getArmSysIdRoutine().quasistatic(direction);
	}

	public Command armSysIdDynamic(SysIdRoutine.Direction direction) {
		return getArmSysIdRoutine().dynamic(direction);
	}

	public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
		return getFlywheelSysIdRoutine().quasistatic(direction);
	}

	public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
		return getFlywheelSysIdRoutine().dynamic(direction);
	}
}
