package frc.team2412.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import java.util.List;

public class TrapAlign {

	// Distance from Middle of Trap used for calculations is 0.8m
	private static final Pose2d[] BLUE_TRAP_POSES_REVISED = {
		new Pose2d(5.68, 4.09, new Rotation2d(180.0)), // CENTERLINE SIDE
		new Pose2d(4.48, 3.40, new Rotation2d(-60.0)), // SOURCE SIDE
		new Pose2d(4.48, 4.78, new Rotation2d(-60.0)) // AMP SIDE
	};

	private static final Pose2d[] RED_TRAP_POSES_REVISED = {
		new Pose2d(11.69, 4.11, new Rotation2d(-60.0)), // CENTERLINE SIDE
		new Pose2d(12.09, 3.42, new Rotation2d(0)), // SOURCE SIDE
		new Pose2d(12.09, 4.8, new Rotation2d(-60.0)), // AMP SIDE
	};

	private static final Pose2d[] BLUE_TRAP_POSES = {
		// trap that faces amp
		new Pose2d(new Translation2d(4.3, 5.14), Rotation2d.fromDegrees(-60)),
		// trap that faces source
		new Pose2d(new Translation2d(4.3, 3.09), Rotation2d.fromDegrees(-300)),
		// trap that faces mid
		// DO THIS ONE FIRST
		// brute force the X lol
		new Pose2d(new Translation2d(5.8, 4.10), Rotation2d.fromDegrees(180))
	};

	private static final Pose2d[] RED_TRAP_POSES = {
		// trap that faces amp
		new Pose2d(new Translation2d(12.3, 5.14), Rotation2d.fromDegrees(-120)),
		// trap that faces source
		new Pose2d(new Translation2d(12.15, 3.45), Rotation2d.fromDegrees(120)),
		// trap that faces mid
		// DO THIS ONE FIRST
		// brute force the X lol
		new Pose2d(new Translation2d(10.8, 4.10), Rotation2d.fromDegrees(0))
	};

	private static Command trapAlign(DrivebaseSubsystem drivebaseSubsystem) {
		Pose2d robotPose = drivebaseSubsystem.getPose();
		boolean isBlue;
		if (!DriverStation.getAlliance().isEmpty()) {
			isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
		} else {
			isBlue = false;
		}
		// figures out which trap to go to
		Pose2d trapPose = robotPose.nearest(List.of(isBlue ? BLUE_TRAP_POSES : RED_TRAP_POSES));
		// sets the point for the path to go to
		List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(robotPose, trapPose);
		// this is flipped
		PathPlannerPath path =
				new PathPlannerPath(
						bezierPoints,
						new PathConstraints(
								DrivebaseSubsystem.MAX_SPEED,
								DrivebaseSubsystem.MAX_ACCELERATION,
								DrivebaseSubsystem.MAX_ANGULAR_VELOCITY,
								DrivebaseSubsystem.MAX_ANGULAR_ACCELERAITON),
						new GoalEndState(0.0, trapPose.getRotation()));
		// path.flipPath(); Returns path except it's flipped
		// this unflips it
		if (!isBlue) {
			path = path.flipPath();
		}
		return AutoBuilder.followPath(path);
	}

	public static Command trapPreset(
			DrivebaseSubsystem drivebaseSubsystem, LauncherSubsystem launcherSubsystem) {
		return new AlignCommand(drivebaseSubsystem, launcherSubsystem);
	}

	private static class AlignCommand extends Command {
		private final DrivebaseSubsystem drivebaseSubsystem;

		@SuppressWarnings("UnusedVariable")
		private final LauncherSubsystem launcherSubsystem;

		private Command trapCommand = null;

		public AlignCommand(
				DrivebaseSubsystem drivebaseSubsystem, LauncherSubsystem launcherSubsystem) {
			this.drivebaseSubsystem = drivebaseSubsystem;
			this.launcherSubsystem = launcherSubsystem;
			addRequirements(launcherSubsystem, drivebaseSubsystem);
		}

		@Override
		public void initialize() {
			trapCommand = trapAlign(drivebaseSubsystem);
			trapCommand.initialize();
			// launcherSubsystem.setAngle(LauncherSubsystem.TRAP_AIM_ANGLE);
			// launcherSubsystem.launch(LauncherSubsystem.TRAP_SHOOT_SPEED_RPM);
		}

		@Override
		public void execute() {
			trapCommand.execute();
		}

		@Override
		public boolean isFinished() {
			return trapCommand.isFinished();
		}

		@Override
		public void end(boolean interrupted) {
			if (interrupted && trapCommand != null) {
				trapCommand.end(true);
			}
			trapCommand = null;
		}
	}
}
