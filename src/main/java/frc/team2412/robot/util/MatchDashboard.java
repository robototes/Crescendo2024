package frc.team2412.robot.util;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

		PathPlannerLogging.setLogTargetPoseCallback(
				(pose) -> {
					field.getObject("target pose").setPose(pose);
					if (!Robot.isReal() && s.autonomousTeleopSubsystem.isEnabled()) {
						// for some reason rotation is weird in sim so I'm just going to fake it
						Rotation2d newRotation =
								s.drivebaseSubsystem.getPose().getRotation().interpolate(pose.getRotation(), 0.1);
						s.drivebaseSubsystem.setPose(
								new Pose2d(s.drivebaseSubsystem.getPose().getTranslation(), newRotation));
					}
				});

		PathPlannerLogging.setLogActivePathCallback(
				(poses) -> {
					field.getObject("path").setPoses(poses);
				});

		tab.add(new FMSWidget()).withPosition(0, 0).withSize(4, 1);
		tab.add(field).withPosition(0, 1).withSize(4, 3);
		Robot r = Robot.getInstance();
		AutonomousField.configureShuffleboardTab(tab, 6, 0, "Available Auto Variants", r::addPeriodic);
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
						() -> s.intakeSubsystem.indexSensorHasNote() || s.intakeSubsystem.feederSensorHasNote())
				.withPosition(6, 4)
				.withSize(2, 1);
		tab.addBoolean("Flywheels At Speed", () -> s.launcherSubsystem.isAtSpeed(400))
				.withPosition(8, 4)
				.withSize(2, 1);
	}
}
