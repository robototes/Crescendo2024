package frc.team2412.robot.util;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPaths {
	public static SequentialCommandGroup testAuto =
			new SequentialCommandGroup(
					AutoLogic.getAutoCommand("TestPath"),
					new ConditionalCommand(
							AutoLogic.getAutoCommand("TestPathTrue"),
							AutoLogic.getAutoCommand("TestPathFalse"),
							() -> false));

	

	// public static SequentialCommandGroup midSpeakerCenterLineN3N2N1 =
	// 		new SequentialCommandGroup(
	// 				AutoLogic.getAutoCommand("MidSpeakerQCenterLineN3"),
	// 				new ConditionalCommand(
	// 						AutoLogic.getAutoCommand("QCenterLineN3LCenterLineN3")
	// 								.andThen(
	// 										AutoLogic.vibrateControllerCommand,
	// 										AutoLogic.getAutoCommand("LCenterLineN1QCenterLineN2")
	// 												.andThen(
	// 														Commands.either(
	// 																AutoLogic.getAutoCommand("QCenterLineN2LCenterLineN2")
	// 																		.andThen(
	// 																				AutoLogic.vibrateControllerCommand,
	// 																				AutoLogic.getAutoCommand(("LCenterLineN2LCenterLineN1"))),
	// 																AutoLogic.getAutoCommand("QCenterLineN2LCenterLineN2"),
	// 																AutoLogic::dummyLogic))),
	// 						AutoLogic.getAutoCommand("QCenterLineN3QCenterLineN2")
	// 								.andThen(
	// 										Commands.either(
	// 												AutoLogic.getAutoCommand("N3QCenterLineN2LCenterLineN2")
	// 														.andThen(
	// 																AutoLogic.vibrateControllerCommand,
	// 																AutoLogic.getAutoCommand("LCenterLineN2LCenterLineN1")),
	// 												AutoLogic.getAutoCommand("N3QCenterLineN2LCenterLineN1"),
	// 												AutoLogic::dummyLogic)),
	// 						AutoLogic::dummyLogic),
	// 				AutoLogic.vibrateControllerCommand);

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
