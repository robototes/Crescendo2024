package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.Subsystems.SubsystemConstants;

/*
 * Initializes and updates shuffleboard match dashboard entries
 */
public class MatchDashboard {

	private final ShuffleboardTab tab = Shuffleboard.getTab("Match");
	private final Field2d field;

	public MatchDashboard(Subsystems s) {
		field = s.drivebaseWrapper.getField();

		tab.add(new FMSWidget()).withPosition(0, 0).withSize(4, 1);
		tab.add(field).withPosition(0, 1).withSize(4, 3);
		Robot r = Robot.getInstance();
		AutonomousField.configureShuffleboardTab(tab, 6, 0, "Available Auto Variants", r::addPeriodic);
		if (SubsystemConstants.INTAKE_ENABLED) {
			tab.addBoolean("Left Intake Running", s.intakeSubsystem::isLeftIntakeRunning)
					.withPosition(0, 4)
					.withSize(2, 1);
			tab.addBoolean("Right Intake Running", s.intakeSubsystem::isRightIntakeRunning)
					.withPosition(2, 4)
					.withSize(2, 1);
			tab.addBoolean("Index Running", s.intakeSubsystem::isIndexRunning)
					.withPosition(4, 4)
					.withSize(2, 1);
			tab.addBoolean(
							"Has Note",
							() ->
									s.intakeSubsystem.indexSensorHasNote() || s.intakeSubsystem.feederSensorHasNote())
					.withPosition(6, 4)
					.withSize(2, 1);
		}
		if (SubsystemConstants.LAUNCHER_ENABLED) {
			tab.addBoolean("Flywheels At Speed", () -> s.launcherSubsystem.isAtSpeed(400))
					.withPosition(8, 4)
					.withSize(2, 1);
		}
	}
}
