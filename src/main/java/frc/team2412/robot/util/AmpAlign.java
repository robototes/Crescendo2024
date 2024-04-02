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

public class AmpAlign {
	private static final Pose2d BLUE_AMP_POSES =
			// amp on the blue side
			new Pose2d(new Translation2d(1.8, 7.3), Rotation2d.fromDegrees(270));

	private static final Pose2d RED_AMP_POSES =
			// amp on the red side
			new Pose2d(new Translation2d(14.5, 7.3), Rotation2d.fromDegrees(270));

	private static Command ampAlign(DrivebaseSubsystem drivebaseSubsystem) {
		Pose2d robotPose = drivebaseSubsystem.getPose();
		boolean isBlue;
		if (!DriverStation.getAlliance().isEmpty()) {
			isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
		} else {
			isBlue = false;
		}
		// figures out which amp to go to
		Pose2d ampPose = (isBlue) ? BLUE_AMP_POSES : RED_AMP_POSES;
		// sets the point for the path to go to
		List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(robotPose, ampPose);
		// this is flipped
		PathPlannerPath path =
				new PathPlannerPath(
						bezierPoints,
						new PathConstraints(
								DrivebaseSubsystem.MAX_SPEED,
								DrivebaseSubsystem.MAX_ACCELERATION,
								DrivebaseSubsystem.MAX_ANGULAR_VELOCITY,
								DrivebaseSubsystem.MAX_ANGULAR_ACCELERAITON),
						new GoalEndState(0.0, ampPose.getRotation()));
		// path.flipPath(); Returns path except it's flipped
		// this unflips it
		if (!isBlue) {
			path = path.flipPath();
		}

		return AutoBuilder.followPath(path);
	}

	public static Command ampPreset(
			DrivebaseSubsystem drivebaseSubsystem, LauncherSubsystem launcherSubsystem) {
		return new AlignCommand(drivebaseSubsystem, launcherSubsystem);
	}

	private static class AlignCommand extends Command {
		private final DrivebaseSubsystem drivebaseSubsystem;
		private final LauncherSubsystem launcherSubsystem;
		private Command ampCommand = null;

		public AlignCommand(
				DrivebaseSubsystem drivebaseSubsystem, LauncherSubsystem launcherSubsystem) {
			this.drivebaseSubsystem = drivebaseSubsystem;
			this.launcherSubsystem = launcherSubsystem;
			addRequirements(launcherSubsystem, drivebaseSubsystem);
		}

		@Override
		public void initialize() {
			ampCommand = ampAlign(drivebaseSubsystem);
			ampCommand.initialize();
			// launcherSubsystem.setAngle(LauncherSubsystem.TRAP_AIM_ANGLE);
			// launcherSubsystem.launch(LauncherSubsystem.TRAP_SHOOT_SPEED_RPM);
		}

		@Override
		public void execute() {
			ampCommand.execute();
		}

		@Override
		public boolean isFinished() {
			return ampCommand.isFinished();
		}

		@Override
		public void end(boolean interrupted) {
			if (interrupted && ampCommand != null) {
				ampCommand.end(true);
			}
			ampCommand = null;
		}
	}
}
