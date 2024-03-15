package frc.team2412.robot.util;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.team2412.robot.Robot;

public class SparkPIDWidget implements NTSendable {

	public final SparkPIDController controller;

	public SparkPIDWidget(SparkPIDController controller, String name) {
		this.controller = controller;
		SendableRegistry.add(this, name);
	}

	@Override
	public void initSendable(NTSendableBuilder builder) {
		if (Robot.getInstance().getRobotType() != Robot.RobotType.PRACTICE) {
			// builder.setSmartDashboardType("PIDController");

			// builder.addDoubleProperty("p", controller::getP, controller::setP);
			// builder.addDoubleProperty("i", controller::getI, controller::setI);
			// builder.addDoubleProperty("d", controller::getD, controller::setD);
			// builder.addDoubleProperty("ff", controller::getFF, controller::setFF);
		}
	}
}
