package frc.team2412.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2412.robot.Controls;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.intake.*;

public class AutoLogic {
	public Robot r = Robot.getInstance();
	private final Subsystems s = r.subsystems;
	private final Controls controls = r.controls;

	public boolean dummyLogic() {
		return true;
	}

	public AutoLogic() {
		registerCommands();
	}

	public void registerCommands() {

		// param: String commandName, Command command
		// NamedCommands.registerCommand();
		// NamedCommands.registerCommand(
		// 		"SetAngleRetract", new SetAngleCommand(s.launcherSubsystem, () -> 0));
		// NamedCommands.registerCommand(
		// 		"IntakeDiagnostic", new IntakeDiagnosticCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand(
		// 		"LauncherDiagnostic", new LauncherDiagnosticCommand(s.launcherSubsystem));
		// NamedCommands.registerCommand("AllIn", new AllInCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("AllReverse", new AllReverseCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("AllStop", new AllStopCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("FeederIn", new FeederInCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("FeederReverse", new FeederReverseCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("FeederStop", new FeederStopCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("IndexIn", new IndexInCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("IndexReverse", new IndexReverseCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("IndexStop", new IndexStopCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("IntakeIn", new IntakeInCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("IntakeReverse", new IntakeReverseCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand("IntakeStop", new IntakeStopCommand(s.intakeSubsystem));
		// NamedCommands.registerCommand(
		// 		"SetAngleLaunch", new SetAngleLaunchCommand(s.launcherSubsystem, 0.1, 0.1));
		// NamedCommands.registerCommand("SetAngle", new SetAngleCommand(s.launcherSubsystem, () ->
		// 0.1));
		// NamedCommands.registerCommand(
		// 		"SetLaunchSpeed", new SetLaunchSpeedCommand(s.launcherSubsystem, 0.1));
		// NamedCommands.registerCommand("StopLauncher", new StopLauncherCommand(s.launcherSubsystem));
		NamedCommands.registerCommand(
				"DummyLaunch", new InstantCommand(() -> controls.vibrateDriveController(0.5)));
		NamedCommands.registerCommand(
				"NoDummyLaunch", new InstantCommand(() -> controls.vibrateDriveController(0)));
	}

	// public Command getConditionalCommand(){}

	public Command getAutonomousCommand(String pathName) {
		// Load the path you want to follow using its name in the GUI
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

		// Create a path following command using AutoBuilder. This will also trigger event markers.
		return AutoBuilder.followPath(path);
	}
}
