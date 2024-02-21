package frc.team2412.robot.util;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPaths {
	public static SequentialCommandGroup topSpeakerCenterLineN1N2AutoLine1 =
			new SequentialCommandGroup(
					AutoLogic.getAutonomousCommand("TopSpeakerQCenterLineN1"),
					new ConditionalCommand(
							new SequentialCommandGroup(
									AutoLogic.getAutonomousCommand("QCenterLineN1LCenterLineN1"),
									AutoLogic.vibrateControllerCommand,
									AutoLogic.getAutonomousCommand("LCenterN1QCenterLineN2"),
									new ConditionalCommand(
											new SequentialCommandGroup(
													AutoLogic.getAutonomousCommand("QCenterLineN2LCenterLineN2"),
													AutoLogic.vibrateControllerCommand),
											AutoLogic.getAutonomousCommand("QCenterLineN2LAutoLineN1"),
											AutoLogic::dummyLogic)),
							new SequentialCommandGroup(
									AutoLogic.getAutonomousCommand("QCenterLineN1QCenterLineN2"),
									new ConditionalCommand(
											new SequentialCommandGroup(
													AutoLogic.getAutonomousCommand("QCenterLineN2LCenterLineN2"),
													AutoLogic.vibrateControllerCommand,
													AutoLogic.getAutonomousCommand("LCenterLineN2LAutoLineN1")),
											AutoLogic.getAutonomousCommand("QCenterLineN2LAutoLineN1"),
											AutoLogic::dummyLogic),
									AutoLogic.vibrateControllerCommand),
							AutoLogic::dummyLogic));

	public AutoPaths() {}
}
