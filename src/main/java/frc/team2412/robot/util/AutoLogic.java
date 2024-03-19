package frc.team2412.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
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

public class AutoLogic {
	public static Robot r = Robot.getInstance();
	public static final Subsystems s = r.subsystems;
	public static final Controls controls = r.controls;

	// in place of launching command cause launcher doesnt exist
	public static SequentialCommandGroup vibrateControllerCommand =
			new SequentialCommandGroup(
					new InstantCommand(() -> controls.vibrateDriveController(0.5)),
					new WaitCommand(1.5),
					new InstantCommand(() -> controls.vibrateDriveController(0.0)));

	/**
	 * Placeholder for vision detect note
	 *
	 * @return true
	 */
	public static boolean dummyLogic() {
		return true;
	}

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
		NamedCommands.registerCommand("AutoLogicTest", AutoPaths.testAuto);
		NamedCommands.registerCommand(
				"MidSpeakerCenterLineN5N4N3", AutoPaths.midSpeakerCenterLineN3N2N1);
		NamedCommands.registerCommand(
				"LowSpeakerCenterLineN5N4N3", AutoPaths.lowSpeakerCenterLineN5N4N3);
		NamedCommands.registerCommand("LowSpeakerCenterLineN5N4", AutoPaths.lowSpeakerCenterLineN5N4);
		NamedCommands.registerCommand(
				"TopSpeakerCenterLineN1N2AutoLine1", AutoPaths.TopSpeakerCenterLineN1N2AutoLine1);
		NamedCommands.registerCommand(
				"TopSpeakerCenterLineN1N2N3", AutoPaths.TopSpeakerCenterLineN1N2N3);
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
}
