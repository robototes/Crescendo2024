package frc.team2412.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.Supplier;

public class SendablePose3d implements Sendable {
	private static final Pose3d defaultPose = new Pose3d(-1, -1, -1, new Rotation3d());
	private Supplier<Pose3d> poseSupplier;

	public SendablePose3d(Supplier<Pose3d> poseSupplier) {
		this.poseSupplier = poseSupplier;
	}

	public SendablePose3d(Pose3d pose) {
		this(() -> pose);
	}

	private Pose3d getPose() {
		Pose3d pose = poseSupplier.get();
		if (pose == null) {
			return defaultPose;
		}
		return pose;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Translation X", () -> getPose().getX(), null);
		builder.addDoubleProperty("Translation Y", () -> getPose().getY(), null);
		builder.addDoubleProperty("Translation Z", () -> getPose().getZ(), null);
		builder.addDoubleProperty(
				"Angle X (deg)", () -> Math.toDegrees(getPose().getRotation().getX()), null);
		builder.addDoubleProperty(
				"Angle Y (deg)", () -> Math.toDegrees(getPose().getRotation().getY()), null);
		builder.addDoubleProperty(
				"Angle Z (deg)", () -> Math.toDegrees(getPose().getRotation().getZ()), null);
	}
}
