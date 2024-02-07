package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class Subsystems {
	public static class SubsystemConstants {

		private static final boolean IS_COMP = Robot.getInstance().isCompetition();

		public static final boolean APRILTAGS_ENABLED = false;
		public static final boolean LIMELIGHT_ENABLED = false;
		public static final boolean CLIMB_ENABLED = false;
		public static final boolean LAUNCHER_ENABLED = false;
		public static final boolean INTAKE_ENABLED = false;
		public static final boolean DRIVEBASE_ENABLED = true;
	}

	public final DrivebaseSubsystem drivebaseSubsystem;
	public final LauncherSubsystem launcherSubsystem;

	public Subsystems() {
		// initialize subsystems here (wow thats wild)
		if (DRIVEBASE_ENABLED) {
			drivebaseSubsystem = new DrivebaseSubsystem();
		}
		if (LAUNCHER_ENABLED) {
			launcherSubsystem = new LauncherSubsystem();
		} else {
			launcherSubsystem = null;
		}
	}
}
