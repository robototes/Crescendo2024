package frc.team2412.robot.util;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPaths {
	public static SequentialCommandGroup topSpeakerAutoLineN1CenterLineN1N2 =
			new SequentialCommandGroup(
					AutoLogic.getAutonomousCommand("TopSpeakerQCenterLineN1"),
					new ConditionalCommand(
							AutoLogic.getAutonomousCommand("QCenterLineN1CenterLineN1"),
							AutoLogic.getAutonomousCommand("QCenterLineN1CenterQLineN2"),
							AutoLogic::dummyLogic),
					AutoLogic.getAutonomousCommand("AutoLineN1"));

	public AutoPaths() {}
}
