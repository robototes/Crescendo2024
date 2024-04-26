package frc.team2412.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.drivebase.DriveToNoteCommand;
import frc.team2412.robot.commands.intake.AllInCommand;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.launcher.AimTowardsSpeakerCommand;
<<<<<<< HEAD
=======
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.PrepFlywheelForLaunchCommand;
>>>>>>> 4473c63ed31b1d2ecb3fe157fcf8e03ccab8136d
import frc.team2412.robot.commands.launcher.SetAngleAmpLaunchCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.util.auto.AutoLogic;
import java.util.List;

public class AutonomousTeleopSubsystem extends SubsystemBase {

	// physical properties
	private static final double BUMPER_WIDTH_LENGTH = 0.8382; // meters
	private static final PathConstraints CONSTRAINTS = new PathConstraints(3., 3., 3., 3.);

	// TODO: constructor for config has option for error spike threshold that leads path to be
	// replanned
	private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);

	// TODO: module speed
	private static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
			new HolonomicPathFollowerConfig(
					DrivebaseSubsystem.AUTO_TRANSLATION_PID,
					DrivebaseSubsystem.AUTO_ROTATION_PID,
					3.0,
					BUMPER_WIDTH_LENGTH,
					REPLANNING_CONFIG);

	private static final double PARK_TIME = 15.;
	private static final double FORCE_PARK_TIME = 5.;

	private static final Translation2d BLUE_STAGE_POSITION = new Translation2d(4.88, 4.09);
	private static final Translation2d RED_STAGE_POSITION = new Translation2d(11.7, 4.09);
	private static final double STAGE_SIZE = 1.2;

	private static final double BLUE_WING_LINE = 5.85;
	private static final double RED_WING_LINE = 10.70;

	private static final Pose2d BLUE_SOURCE_POSE = new Pose2d(13.87, 1.2, new Rotation2d());
	private static final Pose2d RED_SOURCE_POSE = new Pose2d(2.7, 1.2, Rotation2d.fromDegrees(180));
	private static final PathPlannerPath BLUE_SOURCE_ROAM_PATH =
			PathPlannerPath.fromPathFile("BlueSourceRoamPath");
	private static final PathPlannerPath RED_SOURCE_ROAM_PATH =
			PathPlannerPath.fromPathFile("RedSourceRoamPath");

	// These points are the center of the speaker
	private static final Translation2d BLUE_SPEAKER_POSITION = new Translation2d(0.05, 5.53);
	private static final Translation2d RED_SPEAKER_POSITION = new Translation2d(16.55, 5.53);
	// Distance we can safely launch into the speaker from in meters
	private static final double SPEAKER_LAUNCHING_DISTANCE = 3.0;

	// amp
	private static final Pose2d BLUE_AMP_ALIGN_POSE =
			new Pose2d(1.82, 7.40, Rotation2d.fromDegrees(90));
	private static final Pose2d RED_AMP_ALIGN_POSE =
			new Pose2d(14.72, 7.40, Rotation2d.fromDegrees(90));
	private static final Pose2d BLUE_AMP_SCORE_POSE =
			new Pose2d(1.82, 7.63, Rotation2d.fromDegrees(90));
	private static final Pose2d RED_AMP_SCORE_POSE =
			new Pose2d(14.72, 7.63, Rotation2d.fromDegrees(90));

	private static final double ACCELERATION_TOLERANCE = 0.1;

	private static final double COLLISION_DISTANCE = BUMPER_WIDTH_LENGTH + 0.3;

	public enum RobotState {
		IDLE {
			@Override
			public RobotState nextState(AutonomousTeleopSubsystem input) {
				currentCommand = null;
				if (input.inMidgame()) {
					if (input.hasNote()) {
						switch (input.scoringMode) {
							case SPEAKER:
								input.goal = RobotGoal.SCORE_SPEAKER;
							case AMP:
								input.goal = RobotGoal.SCORE_AMP;
						}
					} else {
						input.goal = RobotGoal.PICKUP_NOTE;
					}
				} else {
					if (input.underStage()) {
						return this;
					}
					input.goal = RobotGoal.PARK;
				}
				input.clearCollisionSpots();
				return TRAVELLING;
			}
		},
		TRAVELLING {
			@Override
			public RobotState nextState(AutonomousTeleopSubsystem input) {
				// if the current command is null then this is the first time we're running this stage
				if (currentCommand == null) {
					input.s.launcherSubsystem.setAngle(LauncherSubsystem.RETRACTED_ANGLE);
					input.s.launcherSubsystem.stopLauncher();
					switch (input.goal) {
						case PICKUP_NOTE:
							currentCommand = input.pathfindToSource();
							currentCommand.schedule();
							break;
						case SCORE_SPEAKER:
							currentCommand = input.pathfindToSpeaker();
							currentCommand.schedule();
							if (input.alliance.equals(Alliance.Blue)
									? input.s.drivebaseSubsystem.getPose().getX() < BLUE_WING_LINE
									: input.s.drivebaseSubsystem.getPose().getX() > RED_WING_LINE) {}

							break;
						case SCORE_AMP:
							currentCommand = input.pathfindToAmp();
							currentCommand.schedule();
							break;
						case PARK:
							currentCommand = input.pathfindToStage();
							currentCommand.schedule();
							break;
					}
				}
				if (input.isColliding()) {
					return REROUTING;
				}
				if (!currentCommand.isFinished()) {
					return this;
				}
				currentCommand = null;
				switch (input.goal) {
					case PICKUP_NOTE:
						return SEARCHING;
					case SCORE_SPEAKER:
						return SPEAKER_SCORING;
					case SCORE_AMP:
						return AMP_SCORING;
					case PARK:
					default:
						return IDLE;
				}
			}
		},
		REROUTING {
			@Override
			public RobotState nextState(AutonomousTeleopSubsystem input) {
				input.avoidCollisionSpot();
				return TRAVELLING;
			}
		},
		SEARCHING {
			@Override
			public RobotState nextState(AutonomousTeleopSubsystem input) {
				if (currentCommand == null) {
					currentCommand = input.searchNoteCommand();
					currentCommand.schedule();
				}
				if (currentCommand.isFinished()) {
					return IDLE;
				}
				return this;
			}
		},
		SPEAKER_SCORING {
			@Override
			public RobotState nextState(AutonomousTeleopSubsystem input) {
				if (currentCommand == null) {
					currentCommand = input.scoreSpeaker();
					currentCommand.schedule();
				}
				if (currentCommand.isFinished()) {
					return IDLE;
				}
				return this;
			}
		},
		AMP_SCORING {
			@Override
			public RobotState nextState(AutonomousTeleopSubsystem input) {
				if (currentCommand == null) {
					currentCommand = input.scoreAmpCommand();
					currentCommand.schedule();
				}
				if (currentCommand.isFinished()) {
					return IDLE;
				}
				return this;
			}
		};

		public abstract RobotState nextState(AutonomousTeleopSubsystem input);

		Command currentCommand;
	}

	public enum RobotGoal {
		PICKUP_NOTE,
		SCORE_SPEAKER,
		SCORE_AMP,
		PARK
	}

	public enum ScoringMode {
		SPEAKER,
		AMP
	}

	private final Subsystems s;
	private final ShuffleboardTab tab = Shuffleboard.getTab("Auto Teleop");

	private boolean enabled = false;
	private RobotState state = RobotState.IDLE;
	private RobotGoal goal = RobotGoal.PICKUP_NOTE;
	private ScoringMode scoringMode = ScoringMode.SPEAKER;
	private double matchTimeRemaining;
	private boolean inMatch = false;
	private Alliance alliance = Alliance.Blue;

	private SendableChooser<RobotState> forceState;
	private SendableChooser<RobotGoal> forceGoal;

	// for collision detection
	private Translation2d lastExpectedVelocity = new Translation2d();
	private double lastTimestamp = 0;
	private Translation2d collisionDirection = new Translation2d();

	public AutonomousTeleopSubsystem(Subsystems s) {
		this.s = s;

		// Pathfinder w/ obstacle avoidance
		Pathfinding.setPathfinder(new LocalADStar());

		initData();
	}

	public void start() {
		enabled = true;
		inMatch = !(DriverStation.getMatchType().equals(MatchType.None));
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
		state = RobotState.IDLE;
		Pathfinding.ensureInitialized();
		PathfindingCommand.warmupCommand().withName("PathplannerWarmupCommand").schedule();
	}

	public void stop() {
		if (state.currentCommand != null) {
			state.currentCommand.cancel();
		}
		state.currentCommand = null;
		enabled = false;
	}

	public void setScoringMode(ScoringMode scoringMode) {
		this.scoringMode = scoringMode;
	}

	@Override
	public void periodic() {
		if (enabled) {
			matchTimeRemaining = DriverStation.getMatchTime();
			if (matchTimeRemaining <= FORCE_PARK_TIME) {
				// TODO: force park
				return;
			}
			if (forceGoal.getSelected() != null) {
				goal = forceGoal.getSelected();
			}
			if (forceState.getSelected() != null) {
				state = forceState.getSelected();
			} else {
				state = state.nextState(this);
			}
		}
	}

	public void initData() {
		tab.addBoolean("Enabled", () -> enabled);
		tab.addString("Robot State", () -> state.toString());
		tab.addString("Robot Goal", () -> goal.toString());
		tab.addString(
				"Current Command",
				() -> {
					if (state.currentCommand != null) return state.currentCommand.toString();
					else return "None";
				});
		tab.addDouble("Match Time", () -> matchTimeRemaining);
		tab.addBoolean("Is Match", () -> inMatch);
		tab.addBoolean("Has Note", this::hasNote);
		tab.addBoolean("In Midgame", this::inMidgame);
		tab.addBoolean("Under Stage", this::underStage);
		tab.addBoolean("Is Colliding", this::isColliding);

		forceState = new SendableChooser<RobotState>();
		forceState.addOption("None", null);
		for (RobotState state : RobotState.values()) {
			forceState.addOption(state.name(), state);
		}
		tab.add("Force State", forceState);

		forceGoal = new SendableChooser<RobotGoal>();
		forceGoal.addOption("None", null);
		for (RobotGoal goal : RobotGoal.values()) {
			forceGoal.addOption(goal.name(), goal);
		}
		tab.add("Force Goal", forceGoal);
	}

	public boolean isEnabled() {
		return enabled;
	}

	private boolean inMidgame() {
		return !inMatch || matchTimeRemaining > PARK_TIME;
	}

	private boolean hasNote() {
		return s.intakeSubsystem.feederSensorHasNote() || s.intakeSubsystem.indexSensorHasNote();
	}

	private boolean underStage() {
		Translation2d robotPosition = s.drivebaseSubsystem.getPose().getTranslation();
		if (alliance.equals(Alliance.Blue)) {
			return robotPosition.getDistance(BLUE_STAGE_POSITION) < STAGE_SIZE;
		} else {
			return robotPosition.getDistance(RED_STAGE_POSITION) < STAGE_SIZE;
		}
	}

	public boolean isColliding() {
		Translation2d expectedVelocity =
				new Translation2d(
						s.drivebaseSubsystem.getFieldSpeeds().vxMetersPerSecond,
						s.drivebaseSubsystem.getFieldSpeeds().vyMetersPerSecond);
		// estimate expected acceleration using a = Δv/Δt
		Translation2d expectedAccel =
				(lastExpectedVelocity.minus(expectedVelocity))
						.div((lastTimestamp - Timer.getFPGATimestamp()));
		Translation2d actualAccel =
				s.drivebaseSubsystem
						.getSwerveDrive()
						.getAccel()
						.orElse(new Translation3d(expectedAccel.getX(), expectedAccel.getY(), 0))
						.toTranslation2d();

		double accelDifference = Math.abs(expectedAccel.getNorm() - actualAccel.getNorm());

		lastExpectedVelocity = expectedVelocity;
		lastTimestamp = Timer.getFPGATimestamp();

		if (Math.abs(accelDifference) > ACCELERATION_TOLERANCE) {
			collisionDirection = expectedAccel.minus(actualAccel);
			return true;
		}
		return false;
	}

	public void avoidCollisionSpot() {
		ChassisSpeeds currentSpeeds = s.drivebaseSubsystem.getRobotSpeeds();

		Translation2d currentPosition = s.drivebaseSubsystem.getPose().getTranslation();

		double collisionAngle = Math.tan(collisionDirection.getY() / collisionDirection.getX());

		Translation2d collidingBotPosition =
				new Translation2d(
						currentPosition.getX() + (COLLISION_DISTANCE * Math.cos(collisionAngle)),
						currentPosition.getY() + (COLLISION_DISTANCE * Math.sin(collisionAngle)));

		// collision box x/y size varies on direction of acceleration
		// im having an anuerysmn naming things (hardest part) lol
		double obstacleLengthX = COLLISION_DISTANCE;
		double obstacleLengthY = COLLISION_DISTANCE;

		// vertical leaning collision
		if ((collisionAngle > 45 && collisionAngle < 135)
				|| (collisionAngle > 225 && collisionAngle < 315)) {
			obstacleLengthY *= 1.5;
		} else {
			obstacleLengthX *= 1.5;
		}

		Pair<Translation2d, Translation2d> collisionBox =
				new Pair<Translation2d, Translation2d>(
						collidingBotPosition.plus(new Translation2d(obstacleLengthX, obstacleLengthY)),
						collidingBotPosition.plus(new Translation2d(-obstacleLengthX, -obstacleLengthY)));

		Pathfinding.setDynamicObstacles(List.of(collisionBox), currentPosition);
		// TODO: reset dynamic obstacles after certain condition
		// After certain amount of time?
	}

	public void clearCollisionSpots() {
		Pathfinding.setDynamicObstacles(List.of(), s.drivebaseSubsystem.getPose().getTranslation());
	}

	// Path find methods

	private void setGoalPosition(Translation2d robotPosition, Translation2d goalPosition) {
		Pathfinding.setStartPosition(robotPosition);
		Pathfinding.setGoalPosition(goalPosition);
	}

	// Commands

	private Command pathfindToSource() {
		Pose2d sourcePose = alliance.equals(Alliance.Blue) ? BLUE_SOURCE_POSE : RED_SOURCE_POSE;
		return pathfindToPose(sourcePose);
	}

	private Command pathfindToSpeaker() {
		// We want to travel the minimum distance to the speaker to launch from it
		// To do so, we consider a radius around the center of the circle we can theoretically launch
		// from.
		// Then we project a line from the robot's current position to the center of the speaker and
		// calculate the intersection.
		// Then we pathfind to the intersection, which should be the closest point we can launch from.
		// https://www.desmos.com/calculator/zwqwkgrv5i for visual

		Translation2d speakerCenter =
				alliance.equals(Alliance.Blue) ? BLUE_SPEAKER_POSITION : RED_SPEAKER_POSITION;
		Pose2d robotPose = s.drivebaseSubsystem.getPose();
		Translation2d robotToSpeaker = robotPose.getTranslation().minus(speakerCenter);
		Translation2d closestPoint =
				speakerCenter.plus(
						robotToSpeaker.div(robotToSpeaker.getNorm()).times(SPEAKER_LAUNCHING_DISTANCE));
		Rotation2d closestPointRotation = speakerCenter.minus(closestPoint).getAngle();

		return pathfindToPose(new Pose2d(closestPoint, closestPointRotation));
	}

	private Command pathfindToAmp() {
		Pose2d ampPose = alliance.equals(Alliance.Blue) ? BLUE_AMP_ALIGN_POSE : RED_AMP_ALIGN_POSE;
		return pathfindToPose(ampPose);
	}

	private Command pathfindToStage() {
		return null;
	}

	public Command pathfindToPose(Pose2d goalPose) {
		return new PathfindHolonomic(
				goalPose,
				CONSTRAINTS,
				s.drivebaseSubsystem::getPose,
				s.drivebaseSubsystem::getRobotSpeeds,
				s.drivebaseSubsystem::drive,
				PATH_FOLLOWER_CONFIG,
				s.drivebaseSubsystem);
	}

	public Command searchNoteCommand() {
		Robot robot = Robot.getInstance();
		PathPlannerPath roamPath =
				alliance.equals(Alliance.Blue) ? BLUE_SOURCE_ROAM_PATH : RED_SOURCE_ROAM_PATH;
		return Commands.race(
				AutoBuilder.followPath(roamPath)
						.repeatedly()
						.until(s.limelightSubsystem::hasTargets)
						.andThen(new DriveToNoteCommand(s.drivebaseSubsystem, s.limelightSubsystem)),
				intake());
	}

	public Command scoreAmpCommand() {
		Pose2d ampScorePose = alliance.equals(Alliance.Blue) ? BLUE_AMP_SCORE_POSE : RED_AMP_SCORE_POSE;
		Pathfinding.setGoalPosition(ampScorePose.getTranslation());

		return Commands.sequence(
				new SetAngleAmpLaunchCommand(
						s.launcherSubsystem,
						LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						LauncherSubsystem.AMP_AIM_ANGLE),
				pathfindToPose(ampScorePose),
				Commands.waitUntil(AutoLogic.isReadyToLaunch()).andThen(AutoLogic.feedUntilNoteLaunched()));
	}

	public Command scoreSpeaker() {
		return new AimTowardsSpeakerCommand(s.launcherSubsystem, s.drivebaseSubsystem)
				.andThen(launch());
	}

	public Command launch() {
		return Commands.waitUntil(AutoLogic.isReadyToLaunch())
				.andThen(new FeederInCommand(s.intakeSubsystem).until(AutoLogic.untilFeederHasNoNote()));
	}

	public Command intake() {
		return new SetAngleLaunchCommand(s.launcherSubsystem, 0, LauncherSubsystem.RETRACTED_ANGLE)
				.andThen(new AllInCommand(s.intakeSubsystem, null));
	}

	public Command revFlyWheels() {
		return new PrepFlywheelForLaunchCommand(s.launcherSubsystem, s.drivebaseSubsystem);
	}
}
