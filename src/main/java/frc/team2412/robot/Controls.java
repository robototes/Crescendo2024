package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CODRIVER_CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.LAUNCHER_ENABLED;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.intake.FeederStopCommand;
import frc.team2412.robot.commands.intake.IndexInCommand;
import frc.team2412.robot.commands.intake.FeederReverseCommand;
import frc.team2412.robot.commands.intake.IndexReverseCommand;
import frc.team2412.robot.commands.intake.IndexStopCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeReverseCommand;
import frc.team2412.robot.commands.intake.IntakeStopCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;

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
	private final Trigger driveIntakeSpitButton;
	private final Trigger codriveIntakeInButton;
	private final Trigger codriveIntakeStopButton;
	private final Trigger codriveIntakeSpitButton;
	// Launcher
	private final Trigger launcherAmpPresetButton;
	private final Trigger launcherSubwooferPresetButton;
	private final Trigger launcherPodiumPresetButton;
	private final Trigger launcherTrapPresetButton;

	private final Subsystems s;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;

		launcherAmpPresetButton = codriveController.povDown();
		launcherSubwooferPresetButton = codriveController.povRight();
		launcherPodiumPresetButton = codriveController.povLeft();
		launcherTrapPresetButton = codriveController.povUp();
		// intake buttons (may change later)
		driveIntakeInButton = codriveController.povLeft();
		driveIntakeStopButton = codriveController.povRight();
		driveIntakeSpitButton = codriveController.povUp();
		codriveIntakeInButton = codriveController.x();
		codriveIntakeStopButton = codriveController.b();
		codriveIntakeSpitButton = codriveController.y();

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

	// launcher controls
	private void bindLauncherControls() {
		launcherPodiumPresetButton.onTrue(
				new SetAngleLaunchCommand(
						s.launcherSubsystem,
						LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						LauncherSubsystem.PODIUM_AIM_ANGLE));
		launcherSubwooferPresetButton.onTrue(
				new SetAngleLaunchCommand(
						s.launcherSubsystem,
						LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						LauncherSubsystem.SUBWOOFER_AIM_ANGLE));
	}

	// intake controls
	private void bindIntakeControls() {
		// later on set default command to check sensing notes
		CommandScheduler.getInstance()
				.setDefaultCommand(s.intakeSubsystem, new IntakeStopCommand(s.intakeSubsystem));
		codriveIntakeInButton.onTrue(
				Commands.parallel(
						new IntakeInCommand(s.intakeSubsystem),
						new IndexInCommand(s.intakeSubsystem),
						new FeederInCommand(s.intakeSubsystem)));
		codriveIntakeStopButton.onTrue(
				Commands.parallel(
						new IntakeStopCommand(s.intakeSubsystem),
						new IndexStopCommand(s.intakeSubsystem),
						new FeederStopCommand(s.intakeSubsystem)));
		codriveIntakeSpitButton.onTrue(
				Commands.parallel(
						new IntakeReverseCommand(s.intakeSubsystem),
						new IndexReverseCommand(s.intakeSubsystem),
						new FeederReverseCommand(s.intakeSubsystem)));
	}
}
