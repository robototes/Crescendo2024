package frc.team2412.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Subsystems.SubsystemConstants;
import frc.team2412.robot.commands.LED.LightsCommand;
import frc.team2412.robot.commands.diagnostic.IntakeDiagnosticCommand;
import frc.team2412.robot.commands.diagnostic.LauncherDiagnosticCommand;
import frc.team2412.robot.util.MACAddress;
import frc.team2412.robot.util.MatchDashboard;

public class Robot extends TimedRobot {
	/** Singleton Stuff */
	private static Robot instance = null;

	public enum RobotType {
		COMPETITION,
		PRACTICE,
		CRANE,
		BONK;
	}

	public static Robot getInstance() {
		if (instance == null) instance = new Robot();
		return instance;
	}

	private final RobotType robotType;
	public Controls controls;
	public Subsystems subsystems;
	public MatchDashboard dashboard;

	public SendableChooser<Command> autoChooser;

	protected Robot(RobotType type) {
		// non public for singleton. Protected so test class can subclass
		instance = this;
		robotType = type;
	}

	protected Robot() {
		this(getTypeFromAddress());
	}

	public static final MACAddress COMPETITION_ADDRESS = MACAddress.of(0x00, 0x00, 0x00);
	public static final MACAddress PRACTICE_ADDRESS = MACAddress.of(0x33, 0x9d, 0xD1);
	public static final MACAddress BONK_ADDRESS = MACAddress.of(0x33, 0x9D, 0xE7);
	public static final MACAddress CRANE_ADDRESS = MACAddress.of(0x22, 0xB0, 0x92);

	private static RobotType getTypeFromAddress() {
		if (PRACTICE_ADDRESS.exists()) return RobotType.PRACTICE;
		if (CRANE_ADDRESS.exists()) return RobotType.CRANE;
		if (BONK_ADDRESS.exists()) return RobotType.BONK;
		if (!COMPETITION_ADDRESS.exists())
			DriverStation.reportWarning(
					"Code running on unknown MAC Address! Running competition code anyways", false);
		return RobotType.COMPETITION;
	}

	@Override
	public void robotInit() {
		LiveWindow.disableAllTelemetry();

		subsystems = new Subsystems();
		controls = new Controls(subsystems);

		if (Subsystems.SubsystemConstants.DRIVEBASE_ENABLED) {
			autoChooser = AutoBuilder.buildAutoChooser();
		} else {
			autoChooser = new SendableChooser<>();
		}

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

		dashboard = new MatchDashboard(subsystems);
	}

	@Override
	public void testInit() {
		if (SubsystemConstants.INTAKE_ENABLED) {
			CommandScheduler.getInstance()
					.schedule(new IntakeDiagnosticCommand(subsystems.intakeSubsystem));
		}
		if (SubsystemConstants.LAUNCHER_ENABLED) {
			CommandScheduler.getInstance()
					.schedule(new LauncherDiagnosticCommand(subsystems.launcherSubsystem));
		}
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		Shuffleboard.startRecording();

		// Checks if FMS is attatched and enables joystick warning if true
		DriverStation.silenceJoystickConnectionWarning(!DriverStation.isFMSAttached());

		autoChooser.getSelected().schedule();
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
		Command ledCommand =
				new LightsCommand(
								subsystems.ledSubsystem, subsystems.intakeSubsystem, subsystems.launcherSubsystem)
						.ignoringDisable(true);
		ledCommand.schedule();
		Command coastCommand =
				new WaitCommand(5)
						.andThen(
								new InstantCommand(
										() -> {
											if (DriverStation.isDisabled())
												subsystems.drivebaseSubsystem.setMotorBrake(false);
										}))
						.ignoringDisable(true);
		coastCommand.schedule();
	}

	@Override
	public void disabledExit() {
		subsystems.drivebaseSubsystem.setMotorBrake(true);
	}

	public RobotType getRobotType() {
		return robotType;
	}

	public boolean isCompetition() {
		return getRobotType() == RobotType.COMPETITION;
	}
}
