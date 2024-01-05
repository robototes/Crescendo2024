package frc.team2412.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.util.MACAddress;

public class Robot extends TimedRobot {
	/** Singleton Stuff */
	private static Robot instance = null;

	enum RobotType {
		COMPETITION,
		PRACTICE;
	}

	public static Robot getInstance() {
		if (instance == null) instance = new Robot();
		return instance;
	}

	private final RobotType robotType;
	public Controls controls;
	public Subsystems subsystems;

	protected Robot(RobotType type) {
		// non public for singleton. Protected so test class can subclass
		instance = this;
		robotType = type;
	}

	protected Robot() {
		this(getTypeFromAddress());
	}

	public static final MACAddress COMPETITION_ADDRESS = MACAddress.of(0x33, 0x9d, 0xd1);
	public static final MACAddress PRACTICE_ADDRESS = MACAddress.of(0x33, 0x9D, 0xE7);

	private static RobotType getTypeFromAddress() {
		if (PRACTICE_ADDRESS.exists()) return RobotType.PRACTICE;
		else {return RobotType.COMPETITION;}
	}

	@Override
	public void robotInit() {
		LiveWindow.disableAllTelemetry();

		subsystems = new Subsystems();
		controls = new Controls(subsystems);

		Shuffleboard.startRecording();

		if (RobotBase.isReal()) {
			DataLogManager.start();
			DriverStation.startDataLog(DataLogManager.getLog(), true);
		}

		CommandScheduler.getInstance()
				.onCommandInitialize(
						command -> System.out.println("Command initialized: " + command.getName()));
		CommandScheduler.getInstance()
				.onCommandInterrupt(
						command -> System.out.println("Command interrupted: " + command.getName()));
		CommandScheduler.getInstance()
				.onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

		SmartDashboard.putData(CommandScheduler.getInstance());

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void testInit() {}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		Shuffleboard.startRecording();

		// Checks if FMS is attatched and enables joystick warning if true
		DriverStation.silenceJoystickConnectionWarning(!DriverStation.isFMSAttached());
	}

	@Override
	public void teleopInit() {
		Shuffleboard.startRecording();
	}

	@Override
	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void disabledInit() {
		Shuffleboard.stopRecording();
	}

	public RobotType getRobotType() {
		return robotType;
	}

	public boolean isCompetition() {
		return getRobotType() == RobotType.COMPETITION;
	}
}
