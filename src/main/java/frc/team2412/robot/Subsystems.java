package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import frc.team2412.robot.sensors.AprilTagsProcessor;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LEDSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.subsystems.LimelightSubsystem;
import frc.team2412.robot.util.DrivebaseWrapper;
import java.util.function.BooleanSupplier;

public class Subsystems {
	public static class SubsystemConstants {
		public static final boolean APRILTAGS_ENABLED = true;
		public static final boolean LIMELIGHT_ENABLED = false;
		public static final boolean CLIMB_ENABLED = false;
		public static final boolean LAUNCHER_ENABLED = true;
		public static final boolean INTAKE_ENABLED = true;
		public static final boolean DRIVEBASE_ENABLED = true;
		public static final boolean LED_ENABLED = true;
		public static final boolean USE_APRILTAGS_CORRECTION = true;
	}

	public final DrivebaseWrapper drivebaseWrapper;
	public final DrivebaseSubsystem drivebaseSubsystem;
	public final LauncherSubsystem launcherSubsystem;
	public final LimelightSubsystem limelightSubsystem;
	public final IntakeSubsystem intakeSubsystem;
	public final LEDSubsystem ledSubsystem;
	public final AprilTagsProcessor apriltagsProcessor;
	public final BooleanSupplier rotateToSpeaker;

	public Subsystems() {
		// initialize subsystems here (wow thats wild)
		if (DRIVEBASE_ENABLED) {
			drivebaseSubsystem = new DrivebaseSubsystem();
			drivebaseWrapper = new DrivebaseWrapper(drivebaseSubsystem.getSwerveDrive());
		} else {
			drivebaseSubsystem = null;
			drivebaseWrapper = new DrivebaseWrapper();
		}
		if (APRILTAGS_ENABLED) {
			apriltagsProcessor = new AprilTagsProcessor(drivebaseWrapper);
			rotateToSpeaker = () -> apriltagsProcessor.shouldRotateToSpeaker();
		} else {
			apriltagsProcessor = null;
			rotateToSpeaker = () -> false;
		}
		if (LAUNCHER_ENABLED) {
			launcherSubsystem = new LauncherSubsystem();
		} else {
			launcherSubsystem = null;
		}
		if (LIMELIGHT_ENABLED) {
			limelightSubsystem = new LimelightSubsystem();
		} else {
			limelightSubsystem = null;
		}
		if (INTAKE_ENABLED) {
			intakeSubsystem = new IntakeSubsystem();
		} else {
			intakeSubsystem = null;
		}
		if (LED_ENABLED) {
			ledSubsystem = new LEDSubsystem();
		} else {
			ledSubsystem = null;
		}
	}
}
