package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class SyncPivotRelativeEncoderCommand extends Command {

	private int MAX_SAMPLES = 50; // 6 seconds, 50 per second
	private final double ANGLE_SPEED_TOLERANCE = 0.1;
	private final MedianFilter filter;

	private final LauncherSubsystem launcherSubsystem;

	private int samples;

	public SyncPivotRelativeEncoderCommand(LauncherSubsystem launcherSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		filter = new MedianFilter(MAX_SAMPLES);
		samples = 0;
	}

	@Override
	public void execute() {

		// if the pivot is moving, reset the syncing because now inaccurate.
		if (Math.abs(launcherSubsystem.getAngleSpeed()) >= ANGLE_SPEED_TOLERANCE) {
			filter.reset();
			samples = 0;
			return;
		}

		// add current value to median filter
		filter.calculate(launcherSubsystem.getAngle());
		samples++;
	}

	@Override
	public void end(boolean interrupted) {

		if (samples > MAX_SAMPLES) {
			launcherSubsystem.zeroRelativeEncoder(filter.lastValue());
		}
	}

	@Override
	public boolean isFinished() {
		// stop when finished sampling for avg
		return samples > MAX_SAMPLES;
	}
}
