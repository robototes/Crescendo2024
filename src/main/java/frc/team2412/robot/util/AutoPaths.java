package frc.team2412.robot.util;

import static frc.team2412.robot.util.AutoLogic.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class AutoPaths {
	private static Command registerAuto(
			String autoName, Command command, String... primaryPathNames) {
		PathPlannerAutos.registerAuto(autoName, primaryPathNames);
		return command.withName(autoName);
	}

	// Test Auto

	public static Command testAuto =
			registerAuto(
					"AutoLogicTest",
					new SequentialCommandGroup(
							AutoLogic.getAutoCommand("TestPath"),
							new ConditionalCommand(
									AutoLogic.getAutoCommand("TestPathTrue"),
									AutoLogic.getAutoCommand("TestPathFalse"),
									() -> false)),
					"TestPath",
					"TestPathTrue");

	// Complex Autos

	public static Command TopSpeakerCenterLineN1N2AutoLine1 =
			registerAuto(
					"TopSpeakerCenterLineN1N2AutoLine1",
					Commands.sequence(
							SubwooferLaunchCommand(),
							getAutoCommand("TopSpeakerQCenterLineN1"),
							Commands.either(
									Commands.sequence(
											getAutoCommand("QCenterLineN1LCenterLineN1"),
											VisionLaunchCommand(),
											getAutoCommand("LCenterLineN1QCenterLineN2")),
									Commands.sequence(
											getAutoCommand("QCenterLineN1QCenterLineN2"),
											Commands.either(
													Commands.sequence(
															getAutoCommand("QCenterLineN2LCenterLineN2"),
															VisionLaunchCommand(),
															getAutoCommand("LCenterLineN2LAutoLineN1")),
													Commands.sequence(getAutoCommand("QCenterLineN2LAutoLineN1")),
													() -> true)),
									() -> true),
							VisionLaunchCommand()),
					"TopSpeakerQCenterLineN1",
					"QCenterLineN1LCenterLineN1",
					"LCenterLineN1QCenterLineN2");

	public static Command TopSpeakerCenterLineN1N2N3 =
			registerAuto(
					"TopSpeakerCenterLineN1N2N3",
					Commands.sequence(
							SubwooferLaunchCommand(),
							getAutoCommand("TopSpeakerQCenterLineN1"),
							Commands.either(
									Commands.sequence(
											getAutoCommand("QCenterLineN1LCenterLineN1"),
											VisionLaunchCommand(),
											getAutoCommand("LCenterLineN1QCenterLineN2")),
									Commands.sequence(
											getAutoCommand("QCenterLineN1QCenterLineN2"),
											Commands.either(
													Commands.sequence(
															getAutoCommand("QCenterLineN2LCenterLineN2"),
															VisionLaunchCommand(),
															getAutoCommand("LCenterLineN2LCenterLineN3")),
													Commands.sequence(getAutoCommand("QCenterLineN2LCenterLineN3")),
													() -> true)),
									() -> true),
							VisionLaunchCommand()),
					"TopSpeakerQCenterLineN1",
					"QCenterLineN1LCenterLineN1",
					"LCenterLineN1QCenterLineN2");

	public static Command midSpeakerCenterLineN3N2N1 =
			registerAuto(
					"MidSpeakerCenterLineN5N4N3",
					Commands.sequence(
							SubwooferLaunchCommand(),
							getAutoCommand("MidSpeakerQCenterLineN3"),
							Commands.either(
									Commands.sequence(
											getAutoCommand("QCenterLineN3LCenterLineN3"),
											VisionLaunchCommand(),
											getAutoCommand("LCenterLineN3QCenterLineN2"),
											Commands.either(
													Commands.sequence(
															getAutoCommand("N3QCenterLineN2LCenterLineN2"),
															VisionLaunchCommand(),
															getAutoCommand("N3LCenterLineN2LCenterLineN1")),
													getAutoCommand("QCenterLineN2LCenterLineN1"),
													() -> true)),
									Commands.sequence(
											getAutoCommand("QCenterLineN3QCenterLineN2"),
											getAutoCommand("QCenterLineN2LCenterLineN2")),
									() -> true),
							VisionLaunchCommand()),
					"MidSpeakerQCenterLineN3",
					"QCenterLineN3LCenterLineN3",
					"LCenterLineN3QCenterLineN2",
					"N3QCenterLineN2LCenterLineN2",
					"N3LCenterLineN2LCenterLineN1");

	public static Command lowSpeakerCenterLineN5N4 =
			registerAuto(
					"LowSpeakerCenterLineN5N4",
					Commands.sequence(
							SubwooferLaunchCommand(),
							getAutoCommand("LowSpeakerQCenterLineN5"),
							Commands.either(
									Commands.sequence(
											getAutoCommand("QCenterLineN5LCenterLineN5"),
											Commands.waitSeconds(0.5),
											getAutoCommand("LCenterLineN5LCenterLineN4")),
									Commands.sequence(getAutoCommand("QCenterLineN5LCenterLineN4")),
									() -> false)),
					"LowSpeakerQCenterLineN5",
					"QCenterLineN5LCenterLineN5",
					"LCenterLineN5LCenterLineN4");

	public static Command lowSpeakerCenterLineN5N4N3 =
			registerAuto(
					"LowSpeakerCenterLineN5N4N3",
					Commands.sequence(
							SubwooferLaunchCommand(),
							getAutoCommand("LowSpeakerQCenterLineN5"),
							Commands.either(
									Commands.sequence(
											getAutoCommand("QCenterLineN5QCenterLineN4"),
											Commands.either(
													Commands.sequence(
															getAutoCommand("QCenterLineN4LCenterLineN4"),
															VisionLaunchCommand(),
															getAutoCommand("LCenterLineN4LCenterLineN3")),
													Commands.sequence(getAutoCommand("QCenterLineN4LCenterLineN3")),
													() -> true)),
									Commands.sequence(
											getAutoCommand("QCenterLineN5LCenterLineN4"),
											VisionLaunchCommand(),
											getAutoCommand("LCenterLineN4LCenterLineN3")),
									() -> false),
							VisionLaunchCommand()),
					"LowSpeakerQCenterLineN5",
					"QCenterLineN5QCenterLineN4",
					"QCenterLineN4LCenterLineN4",
					"LCenterLineN4LCenterLineN3");

	// new command getters

	public static Command VisionLaunchCommand() {
		return new FullTargetCommand(s.launcherSubsystem, s.drivebaseSubsystem, controls);
	}

	public static Command SubwooferLaunchCommand() {
		return new SetAngleLaunchCommand(
				s.launcherSubsystem,
				LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
				LauncherSubsystem.SUBWOOFER_AIM_ANGLE);
	}
}
