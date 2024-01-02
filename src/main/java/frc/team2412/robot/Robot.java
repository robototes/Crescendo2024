package frc.team2412.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	/** Singleton Stuff */
	private static Robot instance = null;

	public static Robot getInstance() {
		if (instance == null) instance = new Robot();
		return instance;
	}

	public Controls controls;
	public Subsystems subsystems;

	protected Robot() {
		// non public for singleton. Protected so test class can subclass
		instance = this;
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
}
