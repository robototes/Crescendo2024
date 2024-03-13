package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CODRIVER_CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.LAUNCHER_ENABLED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team2412.robot.commands.intake.AllInCommand;
import frc.team2412.robot.commands.intake.AllReverseCommand;
import frc.team2412.robot.commands.intake.AllStopCommand;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.intake.IntakeRejectCommand;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleAmpLaunchCommand;
import frc.team2412.robot.commands.launcher.SetAngleCommand;
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
	private final Trigger driveIntakeReverseButton;
	private final Trigger driveIntakeRejectButton;
	private final Trigger codriveIntakeInButton;
	private final Trigger codriveIntakeStopButton;
	private final Trigger codriveIntakeReverseButton;
	private final Trigger codriveIntakeRejectButton;
	// Launcher
	private final Trigger launcherAmpPresetButton;
	private final Trigger launcherSubwooferPresetButton;
	// private final Trigger launcherPodiumPresetButton;
	// private final Trigger launcherTrapPresetButton;
	private final Trigger launcherLaunchButton;

	private final Subsystems s;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;

		launcherAmpPresetButton = codriveController.x();
		launcherSubwooferPresetButton = codriveController.a();
		// launcherPodiumPresetButton = codriveController.povLeft();
		// launcherTrapPresetButton = codriveController.y();
		launcherLaunchButton = codriveController.leftBumper();
		// intake controls (confirmed with driveteam)
		driveIntakeInButton = driveController.a();
		driveIntakeStopButton = driveController.b();
		driveIntakeReverseButton = driveController.x();
		driveIntakeRejectButton = driveController.povDown();
		codriveIntakeInButton = codriveController.povUp();
		codriveIntakeStopButton = codriveController.povRight();
		codriveIntakeReverseButton = codriveController.povLeft();
		codriveIntakeRejectButton = codriveController.povDown();

		if (DRIVEBASE_ENABLED) {
			bindDrivebaseControls();
		}
		if (LAUNCHER_ENABLED) {
			bindLauncherControls();
		}
		if (INTAKE_ENABLED) {
			bindIntakeControls();
		}
		if (DRIVEBASE_ENABLED) {
			// temporary controls, not sure what drive team wants
			driveController
					.leftBumper()
					.whileTrue(new FullTargetCommand(s.launcherSubsystem, s.drivebaseSubsystem, this));
			// codriveController
			// 		.rightBumper()
			// 		.whileTrue(
			// 				new FullTargetCommand(
			// 						s.launcherSubsystem,
			// 						s.intakeSubsystem,
			// 						s.drivebaseSubsystem,
			// 						this,
			// 						codriveController.leftBumper()));
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
		// driveController
		// 		.back()
		// 		.onTrue(
		// 				new InstantCommand(
		// 						() ->
		// 								s.drivebaseSubsystem.setPose(
		// 										new Pose2d(new Translation2d(1.3, 5.55), new Rotation2d(180)))));
	}

	// intake controls
	private void bindIntakeControls() {
		// CommandScheduler.getInstance()
		// 		.setDefaultCommand(s.intakeSubsystem, new IntakeStopCommand(s.intakeSubsystem));
		driveIntakeInButton.onTrue(new AllInCommand(s.intakeSubsystem));
		// driveIntakeStopButton.onTrue(new AllStopCommand(s.intakeSubsystem));
		driveIntakeReverseButton.onTrue(new AllReverseCommand(s.intakeSubsystem));
		driveIntakeRejectButton.onTrue(new IntakeRejectCommand(s.intakeSubsystem));
		codriveIntakeInButton.onTrue(new AllInCommand(s.intakeSubsystem));
		codriveIntakeStopButton.onTrue(new AllStopCommand(s.intakeSubsystem));
		codriveIntakeReverseButton.onTrue(new AllReverseCommand(s.intakeSubsystem));
		codriveIntakeRejectButton.onTrue(new IntakeRejectCommand(s.intakeSubsystem));

		// feeder shoot note out
		launcherLaunchButton.whileTrue(new FeederInCommand(s.intakeSubsystem));
		driveController.rightBumper().whileTrue(new FeederInCommand(s.intakeSubsystem));
	}

	private void bindLauncherControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(
						s.launcherSubsystem,
						new SetAngleCommand(
								s.launcherSubsystem,
								() ->
										MathUtil.applyDeadband(codriveController.getLeftY(), 0.1)
												* LauncherSubsystem.ANGLE_MAX_SPEED));

		launcherSubwooferPresetButton.onTrue(
				new SetAngleLaunchCommand(
						s.launcherSubsystem,
						LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						LauncherSubsystem.SUBWOOFER_AIM_ANGLE));
		// launcherPodiumPresetButton.onTrue(
		//		new SetAngleLaunchCommand(
		//				s.launcherSubsystem,
		//				LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
		//				LauncherSubsystem.PODIUM_AIM_ANGLE));
		launcherAmpPresetButton.onTrue(
				new SetAngleAmpLaunchCommand(
						s.launcherSubsystem,
						LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						LauncherSubsystem.AMP_AIM_ANGLE));
		// launcherTrapPresetButton.onTrue(
		//		TrapAlign.trapPreset(s.drivebaseSubsystem, s.launcherSubsystem));

		codriveController.b().whileTrue(s.launcherSubsystem.run(s.launcherSubsystem::stopLauncher));

		// codriveController
		// 		.leftBumper()
		// 		.whileTrue(
		// 				s.launcherSubsystem.runEnd(
		// 						s.launcherSubsystem::launch, s.launcherSubsystem::stopLauncher));

		driveController.b().onTrue(new InstantCommand(() -> s.launcherSubsystem.launch(4000)));
	}

	public void vibrateDriveController(double vibration) {
		if (!DriverStation.isAutonomous()) {
			driveController.getHID().setRumble(RumbleType.kBothRumble, vibration);
		}
	}
}
