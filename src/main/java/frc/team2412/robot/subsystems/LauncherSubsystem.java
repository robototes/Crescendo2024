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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.Robot;
import frc.team2412.robot.util.SparkPIDWidget;
import java.util.Map;
import java.util.Optional;

public class LauncherSubsystem extends SubsystemBase {
	// CONSTANTS

	// HARDWARE
	private static final double PIVOT_GEARING_RATIO = 1.0 / 180.0;
	private static final float PIVOT_SOFTSTOP_FORWARD = 0.93f;
	private static final float PIVOT_SOFTSTOP_BACKWARD = 0.635f;
	private static final float PIVOT_DISABLE_OFFSET = 0.04f;
	private static final int PIVOT_OFFSET = 36;

	// offset stuff
	private static final double ENCODER_DIFFERENCE_TOLERANCE = 0.01;
	private static final double OFFSET_SYNCING_TOLERANCE = 0.06;

	// ANGLE VALUES
	public static final int AMP_AIM_ANGLE = 285 + PIVOT_OFFSET;
	public static final int SUBWOOFER_AIM_ANGLE = 252 + PIVOT_OFFSET;
	public static final int PODIUM_AIM_ANGLE = 238 + PIVOT_OFFSET;
	public static final int TRAP_AIM_ANGLE = 317 + PIVOT_OFFSET;
	public static final double MANUAL_MODIFIER = 0.02;
	public static final double RETRACTED_ANGLE = 242 + PIVOT_OFFSET;
	// offset for FF so parallel to floor is 0
	public static final double FF_PIVOT_OFFSET = 225 + PIVOT_OFFSET;

	// MOTOR VALUES
	// max Free Speed: 6784 RPM
	private static final int MAX_FREE_SPEED_RPM = 6784;
	public static final double ANGLE_TOLERANCE = 5;
	public static final double RPM_TOLERANCE = 500;
	// RPM
	public static final int SPEAKER_SHOOT_SPEED_RPM = 4500;
	public static final int TRAP_SHOOT_SPEED_RPM = 4500;
	public static final int LOBBING_RPM = 4700;
	public static final double ANGLE_MAX_SPEED = 0.5; // percent output
	public static final double MAX_SET_ANGLE_OFFSET = 20;
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
	// private final SparkPIDController launcherAngleTwoPIDController;
	private final ArmFeedforward launcherPivotFF = new ArmFeedforward(0.40434, 0.096771, 0.0056403);
	// arm FF values:
	// Ks: 0.40434
	// Kv: 0.096771
	// Ka: 0.0056403
	private final SparkPIDController launcherTopPIDController;
	private final SparkPIDController launcherBottomPIDController;
	// private final SimpleMotorFeedforward launcherTopFeedforward =
	// 		new SimpleMotorFeedforward(0, 0.11335, 0.048325);
	// private final SimpleMotorFeedforward launcherBottomFeedforward =
	// 		new SimpleMotorFeedforward(0, 0.11238, 0.048209);

	private double rpmSetpoint;
	private double angleSetpoint;
	private double manualAngleSetpoint;
	private boolean ignoreLimits;

	private Optional<Double> relativeEncoderStartPosition;

	private GenericEntry setLauncherSpeedEntry;

	private GenericEntry setLauncherAngleEntry;

	private GenericEntry launcherAngleEntry;

	private GenericEntry launcherSpeedEntry;

	private GenericEntry launcherAngleSpeedEntry;

	private GenericEntry launcherIsAtSpeed;

	private GenericEntry launcherAngleManual;

	private GenericEntry speakerDistanceEntry;

	private GenericEntry setAngleOffsetEntry;

	private GenericEntry angleSetpointEntry;

	private GenericEntry launcherFlywheelSetpointEntry;

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
		manualAngleSetpoint = launcherAngleEncoder.getPosition();

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

		relativeEncoderStartPosition = Optional.empty();

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
		launcherTopMotor.setSmartCurrentLimit(60);
		launcherBottomMotor.setSmartCurrentLimit(60);
		launcherAngleOneMotor.setSmartCurrentLimit(100);
		launcherAngleTwoMotor.setSmartCurrentLimit(100);

		launcherAngleOneMotor.setSoftLimit(
				CANSparkBase.SoftLimitDirection.kForward, PIVOT_SOFTSTOP_FORWARD);
		launcherAngleOneMotor.setSoftLimit(
				CANSparkBase.SoftLimitDirection.kReverse, PIVOT_SOFTSTOP_BACKWARD);
		launcherAngleOneMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
		launcherAngleOneMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

		launcherAngleTwoMotor.follow(launcherAngleOneMotor, true);

		// PID
		launcherAngleOnePIDController.setP(5.2);
		launcherAngleOnePIDController.setI(0);
		launcherAngleOnePIDController.setD(0.066248);
		launcherAngleOnePIDController.setOutputRange(-ANGLE_MAX_SPEED, ANGLE_MAX_SPEED);
		launcherTopPIDController.setP(0.002); // 7.7633E-05);
		launcherTopPIDController.setI(0);
		launcherTopPIDController.setD(0.001);
		launcherTopMotor.setClosedLoopRampRate(0.25);

		launcherBottomPIDController.setP(0.002); // 0.00011722);
		launcherBottomPIDController.setI(0);
		launcherBottomPIDController.setD(0.001);
		launcherBottomMotor.setClosedLoopRampRate(0.25);

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
				rpmSetpoint, ControlType.kVelocity, 0); // launcherTopFeedforward.calculate(speed));
		launcherBottomPIDController.setReference(
				rpmSetpoint, ControlType.kVelocity, 0); // launcherBottomFeedforward.calculate(speed));
	}

	public void ampLaunch(double speed) {
		launcherTopPIDController.setReference(
				-speed, ControlType.kVelocity, 0); // launcherTopFeedforward.calculate(-speed));
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

	/**
	 * Sets the launcher angle, taking the offset into account.
	 *
	 * @param launcherAngle Launcher angle. PIVOT_OFFSET will be added to this.
	 */
	public void setAngleWithOffset(double launcherAngle) {
		setAngle(launcherAngle + PIVOT_OFFSET);
	}

	public void setAngle(double launcherAngle) {
		if (launcherAngle != AMP_AIM_ANGLE) {
			angleSetpoint = launcherAngle + setAngleOffsetEntry.getDouble(0);
		} else {
			angleSetpoint = launcherAngle;
		}
		launcherAngleOnePIDController.setReference(
				Units.degreesToRotations(angleSetpoint),
				ControlType.kPosition,
				0,
				launcherPivotFF.calculate(Units.degreesToRadians(launcherAngle - FF_PIVOT_OFFSET), 0));
		manualAngleSetpoint = Units.degreesToRotations(launcherAngle);
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

	public void restoreLimits() {
		this.ignoreLimits = false;
	}

	public void setAngleManual(double joystickInput, boolean powerControl, boolean ignoreLimits) {
		this.ignoreLimits = ignoreLimits;
		if (powerControl || ignoreLimits) {
			launcherAngleOneMotor.set(ignoreLimits ? joystickInput * 0.3 : joystickInput);
			manualAngleSetpoint =
					MathUtil.clamp(
							Units.degreesToRotations(getAngle()),
							PIVOT_SOFTSTOP_BACKWARD,
							PIVOT_SOFTSTOP_FORWARD);
			return;
		}

		manualAngleSetpoint =
				MathUtil.clamp(
						manualAngleSetpoint + joystickInput * MANUAL_MODIFIER,
						PIVOT_SOFTSTOP_BACKWARD,
						PIVOT_SOFTSTOP_FORWARD);

		if (Units.degreesToRotations(getAngle()) > PIVOT_SOFTSTOP_BACKWARD
				&& Units.degreesToRotations(getAngle()) < PIVOT_SOFTSTOP_FORWARD) {
			launcherAngleOnePIDController.setReference(
					manualAngleSetpoint,
					ControlType.kPosition,
					0,
					launcherPivotFF.calculate(
							Units.degreesToRadians(
									Units.rotationsToDegrees(manualAngleSetpoint) - FF_PIVOT_OFFSET),
							0));
		}
	}

	private void initShuffleboard() {
		if (Robot.isDebugMode()) {
			Shuffleboard.getTab("Launcher")
					.add(new SparkPIDWidget(launcherAngleOnePIDController, "launcherAnglePID"))
					.withPosition(2, 0);
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

		Shuffleboard.getTab("Launcher")
				.addDouble("relative encoder offset", () -> relativeEncoderStartPosition.orElse(0.0));

		Shuffleboard.getTab("Launcher.add")
				.addDouble(
						"relative encoder",
						() ->
								Units.rotationsToDegrees(
										launcherAngleOneMotor.getEncoder().getPosition() * PIVOT_GEARING_RATIO));
		Shuffleboard.getTab("Launcher")
				.addDouble("relative encoder (with offset)", () -> getAngleOneMotorAngle());
		launcherIsAtSpeed =
				Shuffleboard.getTab("Match")
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

		setLauncherAngleEntry =
				Shuffleboard.getTab("Launcher").add("Launcher Angle Setpoint", getAngle()).getEntry();

		launcherAngleManual =
				Shuffleboard.getTab("Launcher")
						.add("Launcher manual angle (rot.)", 0)
						.withPosition(0, 1)
						.withSize(1, 1)
						.withWidget(BuiltInWidgets.kTextView)
						.getEntry();
		speakerDistanceEntry =
				Shuffleboard.getTab("Launcher").add("Speaker dist.", 0).withPosition(1, 2).getEntry();

		setAngleOffsetEntry =
				Shuffleboard.getTab("Match")
						.add("Set Angle Offset", 0)
						.withPosition(4, 3)
						.withSize(2, 1)
						.withWidget(BuiltInWidgets.kNumberSlider)
						.withProperties(Map.of("Min", -MAX_SET_ANGLE_OFFSET, "Max", MAX_SET_ANGLE_OFFSET))
						.getEntry();

		angleSetpointEntry =
				Shuffleboard.getTab("Launcher").add("Angle Setpoint", 0).withPosition(2, 2).getEntry();
		launcherFlywheelSetpointEntry =
				Shuffleboard.getTab("Launcher").add("Flywheel Setpoint", 0).withPosition(4, 5).getEntry();

		var manualModeEntry =
				Shuffleboard.getTab("Launcher")
						.add("Full manual mode", false)
						.withPosition(3, 0)
						.withSize(1, 1)
						.withWidget(BuiltInWidgets.kToggleSwitch)
						.getEntry();
		new Trigger(() -> manualModeEntry.getBoolean(false))
				.whileTrue(
						run(() -> {
									setAngleWithOffset(setLauncherAngleEntry.getDouble(getAngle()));
									launch(setLauncherSpeedEntry.getDouble(SPEAKER_SHOOT_SPEED_RPM));
								})
								.withName("Full Manual"));

		Shuffleboard.getTab("Launcher").addBoolean("Ignoring Limits", () -> ignoreLimits);
	}

	public void updateDistanceEntry(double distance) {
		speakerDistanceEntry.setDouble(distance);
	}

	public double getAngleOneMotorAngle() {
		return Units.rotationsToDegrees(
						launcherAngleOneMotor.getEncoder().getPosition() * PIVOT_GEARING_RATIO)
				+ relativeEncoderStartPosition.orElse(0.0);
	}

	public void zeroRelativeEncoder(double pivotAngle) {

		double currentRelativePosition =
				Units.rotationsToDegrees(
						launcherAngleOneMotor.getEncoder().getPosition() * PIVOT_GEARING_RATIO);
		double offset = pivotAngle - currentRelativePosition;

		if (relativeEncoderStartPosition.isEmpty()
				|| Math.abs(relativeEncoderStartPosition.orElse(0.0) + offset) > OFFSET_SYNCING_TOLERANCE) {
			relativeEncoderStartPosition = Optional.of(offset);
		}
	}

	@Override
	public void periodic() {
		launcherAngleEntry.setDouble(getAngle());
		launcherSpeedEntry.setDouble(getLauncherSpeed());
		launcherAngleSpeedEntry.setDouble(getAngleSpeed());
		launcherIsAtSpeed.setBoolean(isAtSpeed());
		launcherAngleManual.setDouble(manualAngleSetpoint);
		angleSetpointEntry.setDouble(angleSetpoint);
		launcherFlywheelSetpointEntry.setDouble(rpmSetpoint);

		// PIVOT ENCODER SANITY CHECKS
		// compares the relative encoder angle vs the absolute encoder angle
		if (relativeEncoderStartPosition.isEmpty()
				&& Math.abs(getAngle() - getAngleOneMotorAngle()) <= ENCODER_DIFFERENCE_TOLERANCE) {
			if (!ignoreLimits) {
				launcherAngleOneMotor.disable();
			}
			DriverStation.reportError(
					"Pivot encoder deviated too far from motor encoder angle ... .. Reported pivot angle of "
							+ launcherAngleEncoder
							+ " and motor angle of "
							+ getAngleOneMotorAngle()
							+ ". Is overidden: "
							+ ignoreLimits,
					false);
		}

		//
		if (launcherAngleEncoder.getPosition() >= PIVOT_SOFTSTOP_FORWARD + PIVOT_DISABLE_OFFSET
				|| launcherAngleEncoder.getPosition() <= PIVOT_SOFTSTOP_BACKWARD - PIVOT_DISABLE_OFFSET) {
			if (!ignoreLimits) {
				launcherAngleOneMotor.disable();
			}
			DriverStation.reportError(
					"Launcher encoder angle is insane!!!! Reports angle of "
							+ getAngle()
							+ " degrees. Is overridden: "
							+ ignoreLimits,
					false);
		}
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
