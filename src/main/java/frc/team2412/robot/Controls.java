package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CODRIVER_CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.LAUNCHER_ENABLED;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class Controls {
	public static class ControlConstants {
		public static final int CONTROLLER_PORT = 0;
		public static final int CODRIVER_CONTROLLER_PORT = 1;
	}

	private final CommandXboxController driveController;
	private final CommandXboxController codriveController;

	private final Subsystems s;

	// Launcher

	private final Trigger launcherAmpPresetButton;
	private final Trigger launcherSubwooferPresetButton;
	private final Trigger launcherPodiumPresetButton;
	private final Trigger launcherTrapPresetButton;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;
		launcherAmpPresetButton = codriveController.povDown();
		launcherSubwooferPresetButton = codriveController.povRight();
		launcherPodiumPresetButton = codriveController.povLeft();
		launcherTrapPresetButton = codriveController.povUp();

		if (DRIVEBASE_ENABLED) {
			bindDrivebaseControls();
		}

		if (LAUNCHER_ENABLED) {
			bindLauncherControls();
		}
	}

	private void bindDrivebaseControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(
						s.drivebaseSubsystem,
						s.drivebaseSubsystem.driveJoystick(
								driveController::getLeftY,
								driveController::getLeftX,
								() -> Rotation2d.fromRotations(driveController.getRightX())));
		driveController.start().onTrue(new InstantCommand(s.drivebaseSubsystem::resetGyro));
		driveController.rightStick().onTrue(new InstantCommand(s.drivebaseSubsystem::toggleXWheels));
		driveController.leftBumper().whileTrue(new FullTargetCommand(null, s.drivebaseSubsystem));
	}

	private void bindLauncherControls() {
		launcherPodiumPresetButton.onTrue(
				new SetAngleLaunchCommand(
						s.launcherSubsystem,
						() -> LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						() -> LauncherSubsystem.PODIUM_AIM_ANGLE));
		launcherSubwooferPresetButton.onTrue(
				new SetAngleLaunchCommand(
						s.launcherSubsystem,
						() -> LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						() -> LauncherSubsystem.SUBWOOFER_AIM_ANGLE));
	}
}
