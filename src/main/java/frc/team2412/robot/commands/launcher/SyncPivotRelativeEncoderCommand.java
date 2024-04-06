package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class SyncPivotRelativeEncoderCommand extends Command {

	private int MAX_SAMPLES = 300; // 6 seconds, 50 per second
	private final MedianFilter filter;

	private final LauncherSubsystem launcherSubsystem;
	;

	private int samples;

	public SyncPivotRelativeEncoderCommand(
			LauncherSubsystem launcherSubsystem, boolean ignorePrevOffset) {
		this.launcherSubsystem = launcherSubsystem;
		filter = new MedianFilter(300);
	}

	public SyncPivotRelativeEncoderCommand(LauncherSubsystem launcherSubsystem) {
		this(launcherSubsystem, false);
	}

	@Override
	public void execute() {

		// if the arm is moving, reset the syncing because now inaccurate.
		if (Math.abs(launcherSubsystem.getAngleSpeed()) <= 0.1) {
			filter.reset();
			samples = 0;
			return;
		}

		filter.calculate(launcherSubsystem.getAngle());
		samples++;
	}

	@Override
	public void end(boolean interrupted) {
		launcherSubsystem.zeroRelativeEncoder(filter.lastValue());
	}

	@Override
	public boolean isFinished() {
		return samples > MAX_SAMPLES;
	}
}
