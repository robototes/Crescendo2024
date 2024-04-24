package frc.team2412.robot.util.auto;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import static frc.team2412.robot.util.auto.AutoLogic.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.BooleanSupplier;

public class ComplexAutoPaths {

	// Test Auto

	public static Command testAuto =
			registerAuto(
					"AutoLogicTest",
					new SequentialCommandGroup(
							AutoLogic.getAutoCommand("TestPath"),
							new ConditionalCommand(
									AutoLogic.getAutoCommand("TestPathTrue"),
									AutoLogic.getAutoCommand("TestPathFalse"),
									checkForTargets())),
					"TestPath",
					"TestPathTrue");

	// Complex Autos

	// public static Command ampAuto =
	// 		registerAuto(
	// 				"Autoline N1 Centerline N1 N2 N3",
	// 				Commands.parallel(
	// 						new SequentialCommandGroup(
	// 								AutoLogic.subwooferLaunch(),
	// 								AutoLogic.getAutoCommand("AMP L_Preload L_AN1"),
	// 								AutoLogic.visionLaunch2(),
	// 								AutoLogic.getAutoCommand("AMP L_AN1 Q_CN1"),
	// 								conditionalPath(
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("AMP Q_CN1 L_CN1"),
	// 												AutoLogic.visionLaunch2(),
	// 												AutoLogic.getAutoCommand("AMP L_CN1 Q_CN2"),
	// 												conditionalPath(
	// 														new SequentialCommandGroup(
	// 																AutoLogic.getAutoCommand("AMP Q_CN2 L_CN2"),
	// 																AutoLogic.visionLaunch2(),
	// 																AutoLogic.getAutoCommand("AMP L_CN2 L_CN3")),
	// 														AutoLogic.getAutoCommand("AMP Q_CN2 L_CN3"))),
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("AMP Q_CN1 L_CN2"),
	// 												AutoLogic.visionLaunch2(),
	// 												AutoLogic.getAutoCommand("AMP L_CN2 L_CN3"))),
	// 								visionLaunch2()),
	// 						AutoLogic.setFlyWheelSpeaker()));

	// public static Command midAuto =
	// 		registerAuto(
	// 				"Centerline N3 N1 N2",
	// 				Commands.parallel(
	// 						AutoLogic.setFlyWheelSpeaker(),
	// 						new SequentialCommandGroup(
	// 								AutoLogic.subwooferLaunch(),
	// 								AutoLogic.getAutoCommand("MID L_Preload L_AN2"),
	// 								visionLaunch2(),
	// 								AutoLogic.getAutoCommand("MID L_AN2 Q_CN3"),
	// 								conditionalPath(
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("MID Q_CN3 L_CN3"),
	// 												visionLaunch2(),
	// 												AutoLogic.getAutoCommand("MID L_CN3 Q_CN1"),
	// 												conditionalPath(
	// 														new SequentialCommandGroup(
	// 																AutoLogic.getAutoCommand("MID Q_CN1 L_CN1"),
	// 																visionLaunch2(),
	// 																AutoLogic.getAutoCommand("MID L_CN1 L_CN2")),
	// 														AutoLogic.getAutoCommand("MID Q_CN1 L_CN2"))),
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("MID Q_CN3 L_CN2"),
	// 												AutoLogic.getAutoCommand("MID L_CN2 L_CN1"))),
	// 								visionLaunch2())));

	// public static Command sourceAuto =
	// 		registerAuto(
	// 				"Centerline N5 N4 N3",
	// 				Commands.parallel(
	// 						AutoLogic.setFlyWheelSpeaker(),
	// 						new SequentialCommandGroup(
	// 								AutoLogic.subwooferLaunch(),
	// 								AutoLogic.getAutoCommand("SOURCE L_Preload"),
	// 								AutoLogic.visionLaunch2(),
	// 								AutoLogic.getAutoCommand("SOURCE L_Preload Q_CN5"),
	// 								conditionalPath(
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("SOURCE Q_CN5 L_CN5"),
	// 												AutoLogic.visionLaunch2(),
	// 												AutoLogic.getAutoCommand("SOURCE L_CN5 Q_CN4"),
	// 												conditionalPath(
	// 														new SequentialCommandGroup(
	// 																AutoLogic.getAutoCommand("SOURCE Q_CN4 L_CN4"),
	// 																AutoLogic.visionLaunch2(),
	// 																AutoLogic.getAutoCommand("SOURCE L_CN4 L_CN3")),
	// 														AutoLogic.getAutoCommand("SOURCE Q_CN4 L_CN3"))),
	// 										new SequentialCommandGroup(
	// 												AutoLogic.getAutoCommand("SOURCE Q_CN5 L_CN4"),
	// 												AutoLogic.visionLaunch2(),
	// 												AutoLogic.getAutoCommand("SOURCE L_CN4 L_CN3"))),
	// 								AutoLogic.visionLaunch2())));

	// Warning at edu.wpi.first.wpilibj.DriverStation.reportJoystickUnpluggedWarning(DriverStation.java:1364): Joystick Button 3 on port 1 not available, check if controller is plugged in
	// Done serializing CSV
	// Error at com.pathplanner.lib.path.PathPlannerPath.fromPathFile(PathPlannerPath.java:305): Unhandled exception: java.lang.RuntimeException: org.json.simple.parser.ParseException: Unexpected character (<) at position 56.
	//         at com.pathplanner.lib.path.PathPlannerPath.fromPathFile(PathPlannerPath.java:305)
	//         at frc.team2412.robot.util.auto.AutoLogic.getAutoCommand(AutoLogic.java:243)
	//         at frc.team2412.robot.util.auto.ComplexAutoPaths.<clinit>(ComplexAutoPaths.java:38)
	//         at frc.team2412.robot.util.auto.AutoLogic.registerCommands(AutoLogic.java:225)
	//         at frc.team2412.robot.util.auto.AutoLogic.<clinit>(AutoLogic.java:79)
	//         at frc.team2412.robot.Robot.robotInit(Robot.java:92)
	//         at edu.wpi.first.wpilibj.TimedRobot.startCompetition(TimedRobot.java:107)
	//         at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:365)
	//         at edu.wpi.first.wpilibj.RobotBase.lambda$startRobot$0(RobotBase.java:433)
	//         at java.base/java.lang.Thread.run(Thread.java:833)
	
	// new command getters

	private static Command conditionalPath(Command onTrue, Command onFalse) {
		return Commands.either(onTrue, onFalse, checkForTargets());
	}

	public static BooleanSupplier checkForTargets() {
		return (LIMELIGHT_ENABLED ? s.limelightSubsystem::isNoteInFront : () -> true);
	}
}
