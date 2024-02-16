package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CODRIVER_CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.LAUNCHER_ENABLED;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team2412.robot.commands.intake.AllInCommand;
import frc.team2412.robot.commands.intake.AllReverseCommand;
import frc.team2412.robot.commands.intake.AllStopCommand;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.launcher.SetAngleCommand;

public class Controls {
	public static class ControlConstants {
		public static final int CONTROLLER_PORT = 0;
		public static final int CODRIVER_CONTROLLER_PORT = 1;
	}

	private final CommandXboxController driveController;
	private final CommandXboxController codriveController;

	// Intake
	private final Trigger driveIntakeInButton;
	private final Trigger driveIntakeStopButton;
	private final Trigger driveIntakeReverseButton;
	private final Trigger codriveIntakeInButton;
	private final Trigger codriveIntakeStopButton;
	private final Trigger codriveIntakeReverseButton;
	// Launcher
	// private final Trigger launcherAmpPresetButton;
	// private final Trigger launcherSubwooferPresetButton;
	// private final Trigger launcherPodiumPresetButton;
	// private final Trigger launcherTrapPresetButton;
	private final Trigger launcherLaunchButton;

	private final Subsystems s;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;

		// launcherAmpPresetButton = codriveController.povDown();
		// launcherSubwooferPresetButton = codriveController.povRight();
		// launcherPodiumPresetButton = codriveController.povLeft();
		// launcherTrapPresetButton = codriveController.povUp();
		launcherLaunchButton = codriveController.a();
		// intake controls (confirmed with driveteam)
		driveIntakeInButton = driveController.x();
		driveIntakeStopButton = driveController.b();
		driveIntakeReverseButton = driveController.y();
		codriveIntakeInButton = codriveController.povLeft();
		codriveIntakeStopButton = codriveController.povRight();
		codriveIntakeReverseButton = codriveController.povUp();
		if (DRIVEBASE_ENABLED) {
			bindDrivebaseControls();
		}
		if (LAUNCHER_ENABLED) {
			bindLauncherControls();
		}
		if (INTAKE_ENABLED) {
			bindIntakeControls();
		}
	}

	// drivebase
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
	}

	// intake controls
	private void bindIntakeControls() {
		// CommandScheduler.getInstance()
		// 		.setDefaultCommand(s.intakeSubsystem, new IntakeStopCommand(s.intakeSubsystem));
		driveIntakeInButton.onTrue(new AllInCommand(s.intakeSubsystem));
		driveIntakeStopButton.onTrue(new AllStopCommand(s.intakeSubsystem));
		driveIntakeReverseButton.onTrue(new AllReverseCommand(s.intakeSubsystem));
		codriveIntakeInButton.onTrue(new AllInCommand(s.intakeSubsystem));
		codriveIntakeStopButton.onTrue(new AllStopCommand(s.intakeSubsystem));
		codriveIntakeReverseButton.onTrue(new AllReverseCommand(s.intakeSubsystem));

		// feeder shoot note out
		launcherLaunchButton.whileTrue(new FeederInCommand(s.intakeSubsystem));
	}

	private void bindLauncherControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(
						s.launcherSubsystem,
						new SetAngleCommand(s.launcherSubsystem, () -> Units.rotationsToDegrees(codriveController.getLeftY())));
		// launcherPodiumPresetButton.onTrue(
		//		new SetAngleLaunchCommand(
		//				s.launcherSubsystem,
		//				LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
		//				LauncherSubsystem.PODIUM_AIM_ANGLE));
		// launcherSubwooferPresetButton.onTrue(
		//		new SetAngleLaunchCommand(
		//				s.launcherSubsystem,
		//				LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
		//				LauncherSubsystem.SUBWOOFER_AIM_ANGLE));

	}
}
