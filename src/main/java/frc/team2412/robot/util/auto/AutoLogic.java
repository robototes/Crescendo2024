package frc.team2412.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import java.util.List;
import java.util.Map;

public class AutoLogic {
	public static Robot r = Robot.getInstance();
	public static final Subsystems s = r.subsystems;
	public static final Controls controls = r.controls;

	public static enum StartPosition {
		AMP_SIDE_SUBWOOFER(
				" Amp Side Subwoofer",
				new Pose2d(0.73, 6.62, new Rotation2d(Units.degreesToRadians(-120)))),
		MID_SIDE_SUBWOOFER(
				"Mid Side Subwoofer", new Pose2d(1.33, 5.55, new Rotation2d(Units.degreesToRadians(180)))),
		SOURCE_SIDE_SUBWOOFER(
				"Source Side Subwoofer",
				new Pose2d(0.73, 4.46, new Rotation2d(Units.degreesToRadians(120)))),
		MISC("Misc", null);

		String title;
		Pose2d startPose;

		StartPosition(String title, Pose2d startPose) {
			this.title = title;
			this.startPose = startPose;
		}
	};

	// paths lists

	private static List<AutoPath> noPiecePaths =
			List.of(
					// presets
					new AutoPath("Test Path", "DiameterTest"),
					new AutoPath("Stand Still", "PresetSourceSide1Score"),
					new AutoPath("Stand Still", "PresetMid1Score"),
					new AutoPath("Stand Still", "PresetAmpSide1Score"),
					// new AutoPath("Pass Auto Line", "PresetSourceSide1ScorePassAutoLine"),
					new AutoPath("Pass Auto Line", "PresetAmpSide1ScorePassAutoLine"));

	private static List<AutoPath> onePiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1", "PresetAmpSide2Score"),
					new AutoPath("Autoline N2", "PresetMidAutoline2Score"),
					new AutoPath("Autoline N3", "PresetSourceSideAutoline2Score"),
					new AutoPath("CenterLine N5", "PresetSourceSideFar2Score"),
					// vision
					new AutoPath("CenterLine N3 N1", "VisionMidFar2Score", true));

	private static List<AutoPath> twoPiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1", "PresetAmpSideAutoline3Score"),
					new AutoPath("Autoline N2", "PresetMidAutoline3Score"),
					// vision
					new AutoPath("Centerline N5 N4", "VisionSourceSide3Score", true),
					new AutoPath(
							"Autoline N1 Centerline STEAL(N1 N2 N3 N4) N5", "VisionAmpSideFarSteal", true));

	private static List<AutoPath> threePiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N2 N3 N1", "PresetMidAutoline4Score"),
					// vision
					new AutoPath("Autoline N1 CenterLine N1 N2", "VisionAmpSide4Score", true),
					new AutoPath("Autoline N1 N2 N3", "VisionAmpSideAutoLine4Score", true),
					new AutoPath("Autoline N3 N2 N1", "VisionMid4Score", true));

	// gulp map

	private static Map<Integer, List<AutoPath>> commandsMap =
			Map.of(0, noPiecePaths, 1, onePiecePaths, 2, twoPiecePaths, 3, threePiecePaths);

	// vars

	// in place of launching command cause launcher doesnt exist
	public static SequentialCommandGroup vibrateControllerCommand =
			new SequentialCommandGroup(
					new InstantCommand(() -> controls.vibrateDriveController(0.5)),
					new WaitCommand(1.5),
					new InstantCommand(() -> controls.vibrateDriveController(0.0)));

	private static ShuffleboardTab tab = Shuffleboard.getTab("Match");

	private static SendableChooser<StartPosition> startPositionChooser =
			new SendableChooser<StartPosition>();
	private static SendableChooser<AutoPath> availableAutos = new SendableChooser<AutoPath>();
	private static GenericEntry amountGamePiecesEntry;
	private static GenericEntry isVisionEntry;

	public AutoLogic() {
		registerCommands();
	}

	/** Registers commands in PathPlanner */
	public void registerCommands() {
		// param: String commandName, Command command

		// Intake
		NamedCommands.registerCommand("StopIntake", new IntakeStopCommand(s.intakeSubsystem));
		NamedCommands.registerCommand("Intake", new AllInCommand(s.intakeSubsystem));
		NamedCommands.registerCommand(
				"IntakeSensorOverride", new AllInSensorOverrideCommand(s.intakeSubsystem));
		// Launcher
		NamedCommands.registerCommand(
				"VisionLaunch",
				Commands.sequence(
						new FullTargetCommand(s.launcherSubsystem, s.drivebaseSubsystem, controls),
						new FeederInCommand(s.intakeSubsystem)));

		NamedCommands.registerCommand(
				"SubwooferLaunch",
				new SetAngleLaunchCommand(
								s.launcherSubsystem,
								LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
								LauncherSubsystem.SUBWOOFER_AIM_ANGLE)
						.andThen(new WaitCommand(1))
						.andThen(new FeederInCommand(s.intakeSubsystem).andThen(new WaitCommand(0.5))));
		NamedCommands.registerCommand("StopLaunch", new StopLauncherCommand(s.launcherSubsystem));
		NamedCommands.registerCommand(
				"RetractPivot",
				new SetAngleLaunchCommand(s.launcherSubsystem, 0, 0)); // TODO: add retract angle

		// Complex Autos
		NamedCommands.registerCommand("AutoLogicTest", ComplexAutoPaths.testAuto);
		NamedCommands.registerCommand(
				"MidSpeakerCenterLineN5N4N3", ComplexAutoPaths.midSpeakerCenterLineN3N2N1);
		NamedCommands.registerCommand(
				"LowSpeakerCenterLineN5N4N3", ComplexAutoPaths.lowSpeakerCenterLineN5N4N3);
		NamedCommands.registerCommand(
				"TopSpeakerCenterLineN1N2AutoLine1", ComplexAutoPaths.TopSpeakerCenterLineN1N2AutoLine1);
		NamedCommands.registerCommand(
				"TopSpeakerCenterLineN1N2AutoLine1", ComplexAutoPaths.TopSpeakerCenterLineN1N2N3);
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

		tab.add("Starting Position", startPositionChooser).withPosition(5, 1).withSize(2, 1);

		isVisionEntry =
				tab.add("Use Vision Launch", false)
						.withWidget(BuiltInWidgets.kToggleSwitch)
						.withPosition(5, 2)
						.withSize(1, 1)
						.getEntry();
		amountGamePiecesEntry = tab.add("Game Pieces", 0).withPosition(5, 2).withSize(1, 1).getEntry();
		tab.add("Available Auto Variants", availableAutos).withPosition(5, 3).withSize(2, 1);
	}

	/** Takes the auto filtering entries in shuffleboard to provide a list of suitable autos */
	public static void filterAutos() {
		availableAutos.close();

		List<AutoPath> autoCommandsList = commandsMap.get((int) amountGamePiecesEntry.getInteger(0));

		// List<AutoPath> filteredList = new ArrayList<AutoPath>();

		for (AutoPath auto : autoCommandsList) {
			if (auto.getStartPose().equals(startPositionChooser.getSelected().startPose)
					&& auto.isVision() == isVisionEntry.getBoolean(false)) {
				// filteredList.add(auto);
				availableAutos.addOption(auto.getDisplayName(), auto);
			}
		}
	}
}
