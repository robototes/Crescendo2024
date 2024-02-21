package frc.team2412.robot.util;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoPaths {
	public static SequentialCommandGroup topSpeakerAutoLineN1CenterLineN1N2 =
			new SequentialCommandGroup(
					AutoLogic.getAutonomousCommand("TopSpeakerQCenterLineN1"),
					new ConditionalCommand(
							new SequentialCommandGroup(
									AutoLogic.getAutonomousCommand("QCenterLineN1LCenterLineN1"),
									AutoLogic.vibrateControllerCommand,
									AutoLogic.getAutonomousCommand("LCenterN1QCenterLineN2"),
									new ConditionalCommand(
											new SequentialCommandGroup(
													AutoLogic.getAutonomousCommand("QCenterLineN2LCenterLineN2")),
											AutoLogic.getAutonomousCommand("QCenterLineN2LAutoLineN1"),
											AutoLogic::dummyLogic)),
							new SequentialCommandGroup(
									AutoLogic.getAutonomousCommand("QCenterLineN1QCenterLineN2"),
									new ConditionalCommand(
											AutoLogic.getAutonomousCommand("QCenterLine2LCenterLine2"),
											new InstantCommand(),
											AutoLogic::dummyLogic),
									AutoLogic.vibrateControllerCommand),
							AutoLogic::dummyLogic),
					AutoLogic.vibrateControllerCommand);

	public AutoPaths() {}
}
