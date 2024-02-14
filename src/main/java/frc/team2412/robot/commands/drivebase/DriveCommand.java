package frc.team2412.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
	private static final double TURBO_ROTATION_DEFAULT = 1.5;

	private final DrivebaseSubsystem drivebaseSubsystem;
	private final DoubleSupplier forward;
	private final DoubleSupplier strafe;
	private final DoubleSupplier rotation;
	private final DoubleSupplier turboRotation;
	// temporary because code is kinda messy
	Rotation2d MAX_ROTATIONS_PER_SEC = Rotation2d.fromRotations(0.8574);

	// shuffleboard
	private static GenericEntry driveSpeedEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Drive Speed 1", 1)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 1))
					.getEntry();
	private static GenericEntry rotationSpeedEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Rotation Speed 1", 1)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 1))
					.getEntry();
	private static GenericEntry fieldOrientedEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Field Oriented 1", true)
					.withWidget(BuiltInWidgets.kToggleSwitch)
					.getEntry();
	private static GenericEntry cubeSpeedEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Cube Speed 1", true)
					.withWidget(BuiltInWidgets.kToggleSwitch)
					.getEntry();
	private static GenericEntry turboRotationEntry =
			Shuffleboard.getTab("Drivebase")
					.addPersistent("Turbo Rotation Modifier 1", TURBO_ROTATION_DEFAULT)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0.5, "Max", 2.5))
					.getEntry();

	public DriveCommand(
			DrivebaseSubsystem drivebaseSubsystem,
			DoubleSupplier forward,
			DoubleSupplier strafe,
			DoubleSupplier rotation,
			DoubleSupplier turboRotation) {
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.forward = forward;
		this.strafe = strafe;
		this.rotation = rotation;
		// this variable give the right trigger input
		this.turboRotation = turboRotation;

		addRequirements(drivebaseSubsystem);
	}

	@Override
	public void execute() {
		double rotationSpeedModifier =
				rotationSpeedEntry.getDouble(1.0)
						* (1
								- (turboRotation.getAsDouble()
										* (1 - turboRotationEntry.getDouble(TURBO_ROTATION_DEFAULT))));

		double x = deadbandCorrection(-forward.getAsDouble());
		double y = deadbandCorrection(strafe.getAsDouble());
		double rot = deadbandCorrection(-rotation.getAsDouble());

		// math for normalizing and cubing inputs
		double magnitude = Math.pow(Math.min(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), 1), 3);
		double angle = Math.atan2(y, x);
		double cubed_x = magnitude * Math.cos(angle);
		double cubed_y = magnitude * Math.sin(angle);

		drivebaseSubsystem.simpleDrive(
				(cubeSpeedEntry.getBoolean(false) ? cubed_x : x)
						* driveSpeedEntry.getDouble(1.0)
						* 4.4196, // convert from percent to m/s
				(cubeSpeedEntry.getBoolean(false) ? cubed_y : y) * driveSpeedEntry.getDouble(1.0) * 4.4196,
				Rotation2d.fromRotations(
						rot
								* rotationSpeedModifier
								* MAX_ROTATIONS_PER_SEC
										.getRotations())); // convert from percent to rotations per second);
	}

	public double deadbandCorrection(double input) {
		return Math.abs(input) < 0.05 ? 0 : (input - Math.signum(input) * 0.05) / 0.95;
	}
}
