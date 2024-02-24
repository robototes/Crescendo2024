package frc.team2412.robot.util;

import static frc.team2412.robot.util.AutoLogic.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPaths {

	// Test Auto

	public static SequentialCommandGroup testAuto =
			new SequentialCommandGroup(
					AutoLogic.getAutoCommand("TestPath"),
					new ConditionalCommand(
							AutoLogic.getAutoCommand("TestPathTrue"),
							AutoLogic.getAutoCommand("TestPathFalse"),
							() -> false));

	// Complex Autos

	public static Command midSpeakerCenterLineN3N2N1 =
			Commands.sequence(
					getAutoCommand("MidSpeakerQCenterLineN3"),
					Commands.either(
							Commands.sequence(
									getAutoCommand("QCenterLineN3LCenterLineN3"),
									Commands.waitSeconds(0.5),
									getAutoCommand("LCenterLineN3QCenterLineN2"),
									Commands.either(
											Commands.sequence(
													getAutoCommand("N3QCenterLineN2LCenterLineN2"),
													Commands.waitSeconds(0.5),
													getAutoCommand("N3LCenterLineN2LCenterLineN1")),
											getAutoCommand("QCenterLineN2LCenterLineN1"),
											() -> true)),
							Commands.sequence(
									getAutoCommand("QCenterLineN3QCenterLineN2"),
									getAutoCommand("QCenterLineN2LCenterLineN2")),
							() -> true));

	public static Command lowSpeakerCenterLineN5N4N3 =
			Commands.sequence(
					Commands.waitSeconds(0.5),
					getAutoCommand("LowSpeakerQCenterLineN5"),
					Commands.either(
							Commands.sequence(
									getAutoCommand("QCenterLineN5LCenterLineN5"),
									Commands.waitSeconds(0.5),
									getAutoCommand("LCenterLineN5LCenterLineN4")),
							Commands.sequence(
									getAutoCommand("QCenterLineN5LCenterLineN4"),
									Commands.waitSeconds(0.5),
									getAutoCommand("LCenterLineN4LCenterLineN3")),
							() -> false));

	public static Command TopSpeakerCenterLineN1N2AutoLine1 =
			Commands.sequence(
					getAutoCommand("TopSpeakerQCenterLineN1"),
					Commands.either(
							Commands.sequence(
									getAutoCommand("QCenterLineN1LCenterLineN1"),
									Commands.waitSeconds(0.5),
									getAutoCommand("LCenterLineN1QCenterLineN2")),
							Commands.sequence(
									getAutoCommand("QCenterLineN1QCenterLineN2"),
									Commands.either(
											Commands.sequence(
													getAutoCommand("QCenterLineN2LCenterLineN2"),
													Commands.waitSeconds(0.5),
													getAutoCommand("LCenterLineN2LAutoLineN1")),
											Commands.sequence(getAutoCommand("QCenterLineN2LAutoLineN1")),
											() -> true)),
							() -> true));

	public static Command TopSpeakerCenterLineN1N2N3 =
			Commands.sequence(
					getAutoCommand("TopSpeakerQCenterLineN1"),
					Commands.either(
							Commands.sequence(
									getAutoCommand("QCenterLineN1LCenterLineN1"),
									Commands.waitSeconds(0.5),
									getAutoCommand("LCenterLineN1QCenterLineN2")),
							Commands.sequence(
									getAutoCommand("QCenterLineN1QCenterLineN2"),
									Commands.either(
											Commands.sequence(
													getAutoCommand("QCenterLineN2LCenterLineN2"),
													Commands.waitSeconds(0.5),
													getAutoCommand("LCenterLineN2LCenterLineN3")),
											Commands.sequence(getAutoCommand("QCenterLineN2LCenterLineN3")),
											() -> true)),
							() -> true));
}
