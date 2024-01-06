package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import frc.team2412.robot.subsystems.DrivebaseSubsystem;

public class Subsystems {
	public static class SubsystemConstants {
		private static final boolean IS_COMP = Robot.getInstance().isCompetition();
		public static final boolean DRIVEBASE_ENABLED = true;
	}

	public final DrivebaseSubsystem drivebaseSubsystem;

	public Subsystems() {
		// initialize subsystems here (wow thats wild)
		if (DRIVEBASE_ENABLED) {
			drivebaseSubsystem = new DrivebaseSubsystem();
		}
	}
}
