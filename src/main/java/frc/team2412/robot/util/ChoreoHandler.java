package frc.team2412.robot.util;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Robot;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;

public class ChoreoHandler {

	private static final DrivebaseSubsystem drivebaseSubsystem =
			Robot.getInstance().subsystems.drivebaseSubsystem;

	// pid

	private static final PIDController xController = new PIDController(0.1, 0, 0);
	private static final PIDController yController = new PIDController(0.1, 0, 0);
	private static final PIDController rotationController = new PIDController(0.1, 0, 0);

	// constructor

	public ChoreoHandler() {}

	// methods

	private static boolean mirrorTrajectory() {
		return DriverStation.getAlliance().get() == Alliance.Blue;
	}

	public static Command getChoreoCommand(String pathName) {

		return Choreo.choreoSwerveCommand(
				Choreo.getTrajectory(pathName),
				drivebaseSubsystem::getPose,
				xController,
				yController,
				rotationController,
				drivebaseSubsystem::drive,
				ChoreoHandler::mirrorTrajectory,
				drivebaseSubsystem);
	}
}
