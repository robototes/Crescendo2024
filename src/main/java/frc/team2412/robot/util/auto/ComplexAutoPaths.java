package frc.team2412.robot.util.auto;

import static frc.team2412.robot.util.auto.AutoLogic.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class ComplexAutoPaths {

	// Paths Hashmap

	// public static enum AutoRegistry {
	// 	TEST("test auto", 0, testAuto);
	// 	public final String autoName;
	// 	public final int gamePieces;
	// 	public final Command autoCommand;
	// 	AutoRegistry(String autoName, int gamePieces, Command autoCommand) {
	// 		this.autoName = autoName;
	// 		this.gamePieces = gamePieces;
	// 		this.autoCommand = autoCommand;
	// 	}
	// }

	// Test Auto

	public static SequentialCommandGroup testAuto =
			new SequentialCommandGroup(
					AutoLogic.getAutoCommand("TestPath"),
					new ConditionalCommand(
							AutoLogic.getAutoCommand("TestPathTrue"),
							AutoLogic.getAutoCommand("TestPathFalse"),
							() -> false));

	// Complex Autos

	public static final Command TopSpeakerCenterLineN1N2AutoLine1 =
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
					VisionLaunchCommand());

	public static final Command TopSpeakerCenterLineN1N2N3 =
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
					VisionLaunchCommand());

	public static final Command midSpeakerCenterLineN3N2N1 =
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
					VisionLaunchCommand());

	public static final Command lowSpeakerCenterLineN5N4 =
			Commands.sequence(
					SubwooferLaunchCommand(),
					getAutoCommand("LowSpeakerQCenterLineN5"),
					Commands.either(
							Commands.sequence(
									getAutoCommand("QCenterLineN5LCenterLineN5"),
									Commands.waitSeconds(0.5),
									getAutoCommand("LCenterLineN5LCenterLineN4")),
							Commands.sequence(getAutoCommand("QCenterLineN5LCenterLineN4")),
							() -> false));

	public static final Command lowSpeakerCenterLineN5N4N3 =
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
					VisionLaunchCommand());

	// new command getters

	public static final Command VisionLaunchCommand() {
		return new FullTargetCommand(s.launcherSubsystem, s.drivebaseSubsystem, controls);
	}

	public static final Command SubwooferLaunchCommand() {
		return new SetAngleLaunchCommand(
				s.launcherSubsystem,
				LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
				LauncherSubsystem.SUBWOOFER_AIM_ANGLE);
	}
}
