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

	public static Command ampSideConditionalCommand =
			registerAuto(
					"Amp Auto",
					AutoLogic.getAutoCommand("AMP L_Preload L_AN1")
							.andThen(
									AutoLogic.getAutoCommand("AMP L_AN1 Q_CN1")
											.andThen(
													conditionalPath(
															AutoLogic.getAutoCommand("AMP Q_CN1 L_CN1")
																	.andThen(AutoLogic.getAutoCommand("AMP L_CN1 Q_CN2")),
															AutoLogic.getAutoCommand("AMP Q_CN1 L_CN2")
																	.andThen(AutoLogic.getAutoCommand("AMP L_CN2 L_CN3"))))));

	// public static Command AMPAUTO =
	// 		registerAuto(
	// 				"merry christmas",
	// 				new SequentialCommandGroup(
	// 						AutoLogic.subwooferLaunch(),
	// 						AutoLogic.getAutoCommand("AMP L_Preload L_AN1"),
	// 						AutoLogic.feedUntilNoteLaunched(),
	// 						AutoLogic.getAutoCommand("AMP L_AN1 Q_CN1"),
	// 						conditionalPath(
	// 								new SequentialCommandGroup(
	// 										AutoLogic.getAutoCommand("AMP Q_CN1 L_CN1"),
	// 										AutoLogic.feedUntilNoteLaunched(),
	// 										AutoLogic.getAutoCommand("AMP L_CN1 Q_CN2"),
	// 										conditionalPath(
	// 												AutoLogic.getAutoCommand("AMP Q_CN2 L_CN2"),
	// 												AutoLogic.getAutoCommand("AMP Q_CN2 L_CN3"))),
	// 								new SequentialCommandGroup(
	// 										AutoLogic.getAutoCommand("AMP Q_CN1 L_CN2"),
	// 										AutoLogic.feedUntilNoteLaunched(),
	// 										AutoLogic.getAutoCommand("AMP L_CN2 L_CN3")))));
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
