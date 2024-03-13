package frc.team2412.robot.util;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.ObjDoubleConsumer;

public class AutonomousField {
	private static final double DEFAULT_PLAYBACK_SPEED = 2;
	private static final double UPDATE_RATE = 0.05;

	public static void configureShuffleboardTab(
			ShuffleboardTab tab,
			int columnIndex,
			int rowIndex,
			String autoChooserName,
			ObjDoubleConsumer<Runnable> addPeriodic) {
		GenericEntry speedMultiplier =
				tab.add("Auto display speed", DEFAULT_PLAYBACK_SPEED)
						.withWidget(BuiltInWidgets.kNumberSlider)
						.withProperties(Map.of("Min", 0.5, "Max", 2.5))
						.withPosition(columnIndex, rowIndex + 2) // Offset by height of Field2d display
						.withSize(2, 1)
						.getEntry();
		StringSubscriber activeAutoSub =
				NetworkTableInstance.getDefault()
						.getTable("Shuffleboard/Match/Auto Chooser")
						.getStringTopic("active")
						.subscribe("");
		var autonomousField =
				new AutonomousField(() -> speedMultiplier.getDouble(DEFAULT_PLAYBACK_SPEED));
		var watchdog =
				new Watchdog(0.001, () -> DriverStation.reportWarning("auto field loop overrun", false));
		addPeriodic.accept(
				() -> {
					watchdog.reset();
					autonomousField.update(activeAutoSub.get());
					watchdog.addEpoch("AutonomousField.update()");
					watchdog.disable();
					if (watchdog.isExpired()) {
						watchdog.printEpochs();
					}
				},
				UPDATE_RATE);
		tab.add("Selected auto", autonomousField.getField())
				.withPosition(columnIndex, rowIndex)
				.withSize(2, 2);
	}

	// Display
	private final Field2d field = new Field2d();

	// Keeping track of the current trajectory
	private PathPlannerAutos.Auto auto;
	private List<PathPlannerTrajectory> trajectories;
	private int trajectoryIndex = 0;

	// Time
	private final DoubleSupplier speedMultiplier;
	private double lastFPGATime;
	private double lastTrajectoryTimeOffset;

	// Checking for changes
	private Optional<String> lastName = Optional.empty();

	public AutonomousField() {
		this(() -> 1);
	}

	public AutonomousField(double speedMultiplier) {
		this(() -> speedMultiplier);
	}

	public AutonomousField(DoubleSupplier speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
	}

	/**
	 * Returns the {@link Field2d} that this object will update.
	 *
	 * @return The Field2d object.
	 */
	public Field2d getField() {
		return field;
	}

	/**
	 * Calculates the pose to display.
	 *
	 * @param autoName The selected auto name.
	 * @return The pose along the auto trajectory
	 */
	public Pose2d getUpdatedPose(String autoName) {
		double speed = speedMultiplier.getAsDouble();
		double fpgaTime = Timer.getFPGATimestamp();
		if (lastName.isEmpty() || !lastName.get().equals(autoName)) {
			lastName = Optional.of(autoName);
			auto = PathPlannerAutos.getAuto(autoName);
			trajectories = auto.trajectories;
			trajectoryIndex = 0;
			lastFPGATime = fpgaTime;
			lastTrajectoryTimeOffset = 0;
		}
		if (trajectories.isEmpty()) {
			if (auto.startingPose != null) {
				return auto.startingPose;
			}
			return new Pose2d();
		}
		lastTrajectoryTimeOffset += (fpgaTime - lastFPGATime) * speed;
		lastFPGATime = fpgaTime;
		while (lastTrajectoryTimeOffset > trajectories.get(trajectoryIndex).getTotalTimeSeconds()) {
			lastTrajectoryTimeOffset -= trajectories.get(trajectoryIndex).getTotalTimeSeconds();
			if (trajectoryIndex >= trajectories.size()) {
				trajectoryIndex = 0;
			}
		}
		return trajectories
				.get(trajectoryIndex)
				.sample(lastTrajectoryTimeOffset)
				.getTargetHolonomicPose();
	}

	/**
	 * Updates the {@link Field2d} robot pose. If the robot is enabled, does nothing and the
	 * trajectory will restart when the robot is disabled.
	 *
	 * @param autoName The name of the selected PathPlanner autonomous routine.
	 */
	public void update(String autoName) {
		if (DriverStation.isEnabled()) {
			lastName = Optional.empty();
			return;
		}
		field.setRobotPose(getUpdatedPose(autoName));
	}
}
