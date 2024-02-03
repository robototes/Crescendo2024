package frc.team2412.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class FMSWidget implements Sendable {
	public FMSWidget() {
		SendableRegistry.add(this, "FMSInfo");
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("FMSInfo");
	}
}
