package frc.team2412.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutoLogic {
	public static Robot r = Robot.getInstance();
	public static final Subsystems s = r.subsystems;
	public static final Controls controls = r.controls;

	List<AutoPath> onePiecePaths = List.of(new AutoPath("Test Path", "testPath"));

	Map<Integer, List<AutoPath>> commandsMap = Map.of(0, List.of(new AutoPath("Test Path", "testPath")));


	// in place of launching command cause launcher doesnt exist
	public static SequentialCommandGroup vibrateControllerCommand =
			new SequentialCommandGroup(
					new InstantCommand(() -> controls.vibrateDriveController(0.5)),
					new WaitCommand(1.5),
					new InstantCommand(() -> controls.vibrateDriveController(0.0)));

	private static ShuffleboardTab tab = Shuffleboard.getTab("Match");

	public static enum StartPosition {
		AMP_SIDE_SUBWOOFER(),
		MID_SIDE_SUBWOOFER(),
		SOURCE_SIDE_SUBWOOFER();
	};

	private static SendableChooser<StartPosition> startPosition;
	private static SendableChooser<String> availableAutos;
	private static GenericEntry amountGamePiecesEntry;
	private static GenericEntry autoRoutinesEntry;

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
				"TopSpeakerCe,nterLineN1N2AutoLine1", ComplexAutoPaths.TopSpeakerCenterLineN1N2AutoLine1);
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

		tab.add("Starting Position", startPosition).withPosition(5, 1).withSize(2, 1);
		amountGamePiecesEntry = tab.add("Game Pieces", 0).withPosition(5, 2).withSize(2, 1).getEntry();
		tab.add("Available Auto Variants", availableAutos).withPosition(5, 3).withSize(2, 1);
	}

	// public static void updateAvailableAutos() {

	// 	for ()

	// }
}
