package frc.team2412.robot.util;

import static frc.team2412.robot.util.AutoLogic.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPaths {

	private static Command placeHolderCommand = Commands.waitSeconds(0.5);

	public static SequentialCommandGroup testAuto =
			new SequentialCommandGroup(
					AutoLogic.getAutoCommand("TestPath"),
					new ConditionalCommand(
							AutoLogic.getAutoCommand("TestPathTrue"),
							AutoLogic.getAutoCommand("TestPathFalse"),
							() -> false));

	public static Command midSpeakerCenterLineN3N2N1 =
			Commands.sequence(
					vibrateControllerCommand,
					getAutoCommand("MidSpeakerQCenterLineN3"),
					Commands.either(
							Commands.sequence(
									getAutoCommand("QCenterLineN3LCenterLineN3"),
									placeHolderCommand,
									getAutoCommand("LCenterLineN3QCenterLineN2"),
									Commands.either(
											Commands.sequence(
													getAutoCommand("N3QCenterLineN2LCenterLineN2"),
													placeHolderCommand,
													getAutoCommand("N3LCenterLineN2LCenterLineN1")),
											getAutoCommand("QCenterLineN2LCenterLineN1"),
											AutoLogic::dummyLogic)),
							Commands.sequence(
									getAutoCommand("QCenterLineN3QCenterLineN2"),
									getAutoCommand("QCenterLineN2LCenterLineN2")),
							AutoLogic::dummyLogic));

	

	// public static Command TopSpeakerCenterLineN1N2AutoLine1 =
	// Commands.sequence(vibrateControllerCommand)

	// public static SequentialCommandGroup topSpeakerCenterLineN1N2AutoLine1 =
	// 		new SequentialCommandGroup(
	// 				AutoLogic.getAutoCommand("TopSpeakerQCenterLineN1"),
	// 				new ConditionalCommand(
	// 						new SequentialCommandGroup(
	// 								AutoLogic.getAutoCommand("QCenterLineN1LCenterLineN1"),
	// 								AutoLogic.vibrateControllerCommand,
	// 								AutoLogic.getAutoCommand("LCenterLineN1QCenterLineN2"),
	// 								new ConditionalCommand(
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("QCenterLineN2LCenterLineN2"),
	// 												AutoLogic.vibrateControllerCommand),
	// 										AutoLogic.getAutoCommand("QCenterLineN2LAutoLineN1"),
	// 										AutoLogic::dummyLogic)),
	// 						new SequentialCommandGroup(
	// 								AutoLogic.getAutoCommand("QCenterLineN1QCenterLineN2"),
	// 								new ConditionalCommand(
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("QCenterLineN2LCenterLineN2"),
	// 												AutoLogic.vibrateControllerCommand,
	// 												AutoLogic.getAutoCommand("LCenterLineN2LAutoLineN1")),
	// 										AutoLogic.getAutoCommand("QCenterLineN2LAutoLineN1"),
	// 										AutoLogic::dummyLogic),
	// 								AutoLogic.vibrateControllerCommand),
	// 						AutoLogic::dummyLogic));

	public AutoPaths() {}
}
