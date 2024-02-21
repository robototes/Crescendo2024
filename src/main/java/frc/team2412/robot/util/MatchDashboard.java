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
		tab.add("Auto Chooser", r.autoChooser).withPosition(4, 0).withSize(2, 1);
	}
}
