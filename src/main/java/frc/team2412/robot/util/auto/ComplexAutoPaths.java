package frc.team2412.robot.util.auto;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import static frc.team2412.robot.util.auto.AutoLogic.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import java.util.function.BooleanSupplier;

public class ComplexAutoPaths {

	// Test Auto

	public static Command testAuto =
			registerAuto(
					"AutoLogicTest",
					new SequentialCommandGroup(
							AutoLogic.getAutoCommand("TestPath"),
							new ConditionalCommand(
									AutoLogic.getAutoCommand("TestPathTrue"),
									AutoLogic.getAutoCommand("TestPathFalse"),
									checkForTargets())),
					"TestPath",
					"TestPathTrue");

	// Complex Autos

	

	// new command getters

	private static Command conditionalPath(Command onTrue, Command onFalse) {
		return Commands.either(onTrue, onFalse, checkForTargets());
	}

	public static final Command VisionLaunchCommand() {
		return (LAUNCHER_ENABLED && INTAKE_ENABLED && APRILTAGS_ENABLED
				? new FullTargetCommand(s.launcherSubsystem, s.drivebaseSubsystem, controls)
				: Commands.none());
	}

	public static final Command SubwooferLaunchCommand() {
		return (LAUNCHER_ENABLED
				? new SetAngleLaunchCommand(
						s.launcherSubsystem,
						LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						LauncherSubsystem.SUBWOOFER_AIM_ANGLE)
				: Commands.none());
	}

	public static BooleanSupplier checkForTargets() {
		return (LIMELIGHT_ENABLED ? s.limelightSubsystem::isNoteInFront : () -> true);
	}
}
