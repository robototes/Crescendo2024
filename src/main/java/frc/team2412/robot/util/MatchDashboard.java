package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;

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
		tab.add("Left Intake Running", s.intakeSubsystem.isLeftIntakeRunning()).withPosition(0, 4).withSize(2, 1);
		tab.add("Front Intake Running", s.intakeSubsystem.isFrontIntakeRunning()).withPosition(2, 4).withSize(2, 1);
		tab.add("Right Intake Running", s.intakeSubsystem.isRightIntakeRunning()).withPosition(4, 4).withSize(2, 1);
		tab.add("Index Running", s.intakeSubsystem.isIndexRunning()).withPosition(6, 4).withSize(2, 1);
		tab.add("Has Note", s.intakeSubsystem.indexSensorHasNote() || s.intakeSubsystem.feederSensorHasNote()).withPosition(8, 4).withSize(2, 1);
		tab.add("Flywheels At Speed", s.launcherSubsystem.isAtSpeed(400)).withPosition(10, 4).withSize(2, 1);
	}
}
