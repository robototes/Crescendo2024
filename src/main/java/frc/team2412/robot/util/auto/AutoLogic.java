package frc.team2412.robot.util.auto;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Controls;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.intake.AllInCommand;
import frc.team2412.robot.commands.intake.AllInSensorOverrideCommand;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.intake.IntakeStopCommand;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.commands.launcher.StopLauncherCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.util.DynamicSendableChooser;
import frc.team2412.robot.util.PathPlannerAutos;
import frc.team2412.robot.util.PathPlannerAutos.Auto;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class AutoLogic {
	public static Robot r = Robot.getInstance();
	public static final Subsystems s = r.subsystems;
	public static final Controls controls = r.controls;

	public static final double FEEDER_DELAY = 0.4;
	public static final double STAGE_ANGLE = 247;

	public static enum StartPosition {
		AMP_SIDE_SUBWOOFER(
				"Amp Side Subwoofer", new Pose2d(0.73, 6.62, new Rotation2d(Units.degreesToRadians(-120)))),
		MID_SIDE_SUBWOOFER(
				"Mid Side Subwoofer", new Pose2d(1.33, 5.55, new Rotation2d(Units.degreesToRadians(180)))),
		SOURCE_SIDE_SUBWOOFER(
				"Source Side Subwoofer",
				new Pose2d(0.73, 4.47, new Rotation2d(Units.degreesToRadians(120)))),
		MISC("Misc", null);

		final String title; // for shuffleboard display
		final Pose2d startPose; // for identifying path's starting positions for filtering

		StartPosition(String title, Pose2d startPose) {
			this.title = title;
			this.startPose = startPose;
		}
	};

	// TODO: might be a duplicate, keep until after comp
	static {
		if (DRIVEBASE_ENABLED) {
			registerCommands();
		}
	}

	// paths lists

	private static AutoPath defaultPath = new AutoPath("do nothing", "nothing");

	private static List<AutoPath> noPiecePaths =
			List.of(
					// presets
					new AutoPath("Test Path Rotate", "5mForwardRotate180"),
					new AutoPath("Test Path", "DiameterTest"),
					new AutoPath("Tune Translational PID", "TuneTranslationalPID"),
					new AutoPath("Tune Rotational PID", "TuneRotationalPID"),
					new AutoPath("Stand Still", "PresetSourceSide1Score"),
					new AutoPath("Stand Still", "PresetMid1Score"),
					new AutoPath("Stand Still", "PresetAmpSide1Score"),
					new AutoPath("Subwoofer Launch Test", "SubwooferLaunchTest"),
					// new AutoPath("Pass Auto Line", "PresetSourceSide1ScorePassAutoLine"),
					new AutoPath("Pass Autoline", "PresetAmpSide1ScorePassAutoline"),
					new AutoPath("Pass Autoline", "PresetSourceSide1ScorePassAutoline"),
					new AutoPath("Vision Launch Test", "VisionLaunchTest", true));

	private static List<AutoPath> onePiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1", "PresetAmpSide2Score"),
					new AutoPath("Autoline N2", "PresetMidAutoline2Score"),
					new AutoPath("Autoline N3", "PresetSourceSideAutoline2Score"),
					new AutoPath("Centerline N5", "PresetSourceSideFar2Score")
					// vision
					);

	private static List<AutoPath> twoPiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1 Centerline N1", "PresetAmpSideAutoline3Score"),
					new AutoPath("Autoline N2 N1", "PresetMidAutoline3Score"),
					new AutoPath("Centerline N5 N4", "PresetSourceSideCenterline3Score2"),
					new AutoPath("Centerline N5 N3", "PresetSourceSideCenterline3Score2"),
					// vision
					new AutoPath("Centerline N5 N4", "VisionSourceSide3Score", true),
					new AutoPath("Centerline N3 N1", "VisionMidFar2Score", true));

	private static List<AutoPath> threePiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1 N2 N3", "PresetAmpSideAutoline4Score"),
					new AutoPath("Autoline N2 N3 N1", "PresetMidAutoline4Score"),
					new AutoPath("Autoline N3 N2 N1", "PresetSourceSideAutoline4Score"),
					new AutoPath("Autoline N1 Centerline N1 Autoline N2", "PresetAmpSideAutolineFar4Score"),
					// vision
					new AutoPath("Autoline N1 Centerline N1 N2", "VisionAmpSide4Score", true),
					new AutoPath("Autoline N1 N2 N3", "VisionAmpSideAutoline4Score", true),
					new AutoPath("Autoline N3 N2 N1", "VisionMid4Score", true),
					new AutoPath("Autoline N2 Centerline N3 N1", "VisionMidFar4Score2", true),
					new AutoPath("Autoline N2 Centerline N3 N2", "VisionMidFar4Score3", true),
					new AutoPath("Autoline N3 N2 N1", "VisionSourceSideAutoline4Score", true));

	private static List<AutoPath> fourPiecePaths =
			List.of(
					// presets
					// vision
					new AutoPath("Centerline N1 Autoline N1 N2 N3", "VisionAmpSideAutoline5Score", true),
					new AutoPath("Autoline N1 Centerline N1 N2 Autoline N2", "VisionAmpSide5Score", true));

	private static List<AutoPath> fivePiecePaths;

	// map (gulp)
	private static Map<Integer, List<AutoPath>> commandsMap =
			Map.of(
					0,
					noPiecePaths,
					1,
					onePiecePaths,
					2,
					twoPiecePaths,
					3,
					threePiecePaths,
					4,
					fourPiecePaths);

	// vars

	// in place of launching command cause launcher doesnt exist
	public static SequentialCommandGroup vibrateControllerCommand =
			new SequentialCommandGroup(
					new InstantCommand(() -> controls.vibrateDriveController(0.5)),
					new WaitCommand(1.5),
					new InstantCommand(() -> controls.vibrateDriveController(0.0)));

	// shuffleboard
	private static ShuffleboardTab tab = Shuffleboard.getTab("Match");

	private static SendableChooser<StartPosition> startPositionChooser =
			new SendableChooser<StartPosition>();
	private static DynamicSendableChooser<AutoPath> availableAutos =
			new DynamicSendableChooser<AutoPath>();
	private static SendableChooser<Integer> gameObjects = new SendableChooser<Integer>();
	private static SendableChooser<Boolean> isVision = new SendableChooser<Boolean>();

	private static GenericEntry autoDelayEntry;

	// methods

	public static Command registerAuto(String autoName, Command command, String... primaryPathNames) {
		PathPlannerAutos.registerAuto(autoName, primaryPathNames);
		return command.withName(autoName);
	}

	/** Registers commands in PathPlanner */
	public static void registerCommands() {
		// param: String commandName, Command command

		// Intake
		NamedCommands.registerCommand(
				"StopIntake",
				(INTAKE_ENABLED ? new IntakeStopCommand(s.intakeSubsystem) : Commands.none()));
		NamedCommands.registerCommand(
				"Intake",
				(INTAKE_ENABLED
						? Commands.parallel(
								new AllInCommand(s.intakeSubsystem, null),
								new SetAngleLaunchCommand(
										s.launcherSubsystem, 0, LauncherSubsystem.RETRACTED_ANGLE))
						: Commands.none()));
		NamedCommands.registerCommand(
				"IntakeSensorOverride",
				(INTAKE_ENABLED ? new AllInSensorOverrideCommand(s.intakeSubsystem) : Commands.none()));
		// Launcher
		NamedCommands.registerCommand(
				"VisionLaunch",
				(LAUNCHER_ENABLED && INTAKE_ENABLED && APRILTAGS_ENABLED
						? Commands.sequence(
								new FullTargetCommand(s.launcherSubsystem, s.drivebaseSubsystem, controls)
										.until(
												() -> (s.launcherSubsystem.isAtAngle() && s.launcherSubsystem.isAtSpeed()))
										.andThen(new WaitCommand(FEEDER_DELAY)),
								new FeederInCommand(s.intakeSubsystem)
										.until(() -> !s.intakeSubsystem.feederSensorHasNote()))
						: Commands.none()));

		NamedCommands.registerCommand(
				"SubwooferLaunch",
				(LAUNCHER_ENABLED && INTAKE_ENABLED
						? new SetAngleLaunchCommand(
										s.launcherSubsystem,
										LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
										LauncherSubsystem.SUBWOOFER_AIM_ANGLE)
								.until(() -> (s.launcherSubsystem.isAtAngle() && s.launcherSubsystem.isAtSpeed()))
								.andThen(new WaitCommand(FEEDER_DELAY))
								.andThen(new FeederInCommand(s.intakeSubsystem))
								.until(() -> !s.intakeSubsystem.feederSensorHasNote())
								.andThen(new WaitCommand(0.4))
						: Commands.none()));
		NamedCommands.registerCommand(
				"StopLaunch",
				(LAUNCHER_ENABLED ? new StopLauncherCommand(s.launcherSubsystem) : Commands.none()));
		NamedCommands.registerCommand(
				"RetractPivot",
				(LAUNCHER_ENABLED && INTAKE_ENABLED
						? new SetAngleLaunchCommand(s.launcherSubsystem, 0, LauncherSubsystem.RETRACTED_ANGLE)
						: Commands.none())); // TODO: add retract angle

		NamedCommands.registerCommand(
				"UnderStage", new SetAngleLaunchCommand(s.launcherSubsystem, 0, STAGE_ANGLE));

		// Complex Autos
		NamedCommands.registerCommand("AutoLogicTest", ComplexAutoPaths.testAuto);

		NamedCommands.registerCommand(
				"MidSpeakerCenterLineN3N2N1", ComplexAutoPaths.midSpeakerCenterLineN3N2N1);
		NamedCommands.registerCommand(
				"LowSpeakerCenterLineN5N4N3", ComplexAutoPaths.lowSpeakerCenterLineN5N4N3);
		NamedCommands.registerCommand(
				"LowSpeakerCenterLineN5N4", ComplexAutoPaths.lowSpeakerCenterLineN5N4);
		NamedCommands.registerCommand(
				"TopSpeakerCenterLineN1N2AutoLineN1", ComplexAutoPaths.TopSpeakerCenterLineN1N2AutoLineN1);
		NamedCommands.registerCommand(
				"TopSpeakerCenterLineN1N2N3", ComplexAutoPaths.TopSpeakerCenterLineN1N2N3);
	}

	// public Command getConditionalCommand(){}

	/**
	 * Takes a PathPlanner path and returns it as a command.
	 *
	 * @param pathName
	 * @return follow path command
	 */
	public static Command getAutoCommand(String pathName) {
		// Load the path you want to follow using its name in the GUI
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

		// Create a path following command using AutoBuilder. This will also trigger event markers.
		return AutoBuilder.followPath(path);
	}

	public static void initShuffleBoard() {

		startPositionChooser.setDefaultOption(StartPosition.MISC.title, StartPosition.MISC);
		for (StartPosition startPosition : StartPosition.values()) {
			startPositionChooser.addOption(startPosition.title, startPosition);
		}
		isVision.setDefaultOption("Presets", false);
		isVision.addOption("Vision", true);
		gameObjects.setDefaultOption("0", 0);
		for (int i = 1; i < commandsMap.size(); i++) {
			gameObjects.addOption(String.valueOf(i), i);
		}

		tab.add("Starting Position", startPositionChooser).withPosition(5, 0).withSize(2, 1);
		tab.add("Launch Type", isVision).withPosition(5, 1);
		tab.add("Game Objects", gameObjects).withPosition(6, 1);
		tab.add("Available Auto Variants", availableAutos).withPosition(5, 2).withSize(2, 1);
		// autoDelayEntry = tab.add("Auto Delay", 0).withPosition(5, 3).withSize(1, 1).getEntry();

		isVision.onChange((dummyVar) -> AutoLogic.filterAutos(gameObjects.getSelected()));
		startPositionChooser.onChange((dummyVar) -> AutoLogic.filterAutos(gameObjects.getSelected()));
		gameObjects.onChange((dummyVar) -> AutoLogic.filterAutos(gameObjects.getSelected()));

		filterAutos(gameObjects.getSelected());
	}

	/** Takes the auto filtering entries in shuffleboard to provide a list of suitable autos */
	public static void filterAutos(int numGameObjects) {

		// resets/clears all options
		availableAutos.clearOptions();

		availableAutos.setDefaultOption(defaultPath.getDisplayName(), defaultPath);

		// filter based off gameobejct count
		List<AutoPath> autoCommandsList = commandsMap.get(numGameObjects);

		// filter more then add to chooser
		for (AutoPath auto : autoCommandsList) {
			if (auto.getStartPose().equals(startPositionChooser.getSelected())
					&& auto.isVision() == isVision.getSelected()) {
				availableAutos.addOption(auto.getDisplayName(), auto);
			}
		}
	}

	// get auto

	public static Optional<String> getSelectedAutoName() {
		if (availableAutos.getSelected() == null) {
			return Optional.empty();
		}
		return Optional.of(availableAutos.getSelected().getAutoName());
	}

	public static boolean chooserHasAutoSelected() {
		return availableAutos.getSelected() != null;
	}

	public static Command getSelectedAuto() {
		return availableAutos.getSelected().getAutoCommand();
	}

	/**
	 * Takes all of the trajectories of an auto to find the total estimated duration of an auto
	 *
	 * @return auto duration in seconds
	 */
	public static double getEstimatedAutoDuration() {
		if (getSelectedAutoName().isPresent()) {

			Auto auto = PathPlannerAutos.getAuto(getSelectedAutoName().get());
			double autoTime = 0;

			for (PathPlannerTrajectory trajectory : auto.trajectories) {
				autoTime += trajectory.getTotalTimeSeconds();
			}

			// TODO: more accurate estimating by viewing named commands involved

			// rounds time to two decimals
			autoTime *= 100;
			autoTime = ((double) ((int) autoTime)) / 100;

			// add autoDelay to estimation as well
			// autoTime += autoDelayEntry.getDouble(0);

			return autoTime;
		}
		return 0;
	}
}
