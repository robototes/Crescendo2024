package frc.team2412.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.drivebase.DriveToNoteCommand;
import frc.team2412.robot.commands.intake.AllInCommand;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleAmpLaunchCommand;
import frc.team2412.robot.subsystems.AutonomousTeleopSubsystem.RobotState;
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

	private static final Translation2d BLUE_STAGE_POSITION = new Translation2d(4.88, 4.05);
	private static final Translation2d RED_STAGE_POSITION = new Translation2d(11.7, 4.05);
	private static final double STAGE_SIZE = 1.2;

	// TODO: get these points
	private static final Pose2d BLUE_SOURCE_POSE = new Pose2d();
	private static final Pose2d RED_SOURCE_POSE = new Pose2d();
	private static final Pose2d BLUE_SOURCE_ROAM_POSE = new Pose2d();
	private static final Pose2d RED_SOURCE_ROAM_POSE = new Pose2d();

	// amp
	private static final Pose2d BLUE_AMP_ALIGN_POSE = new Pose2d(1.82, 7.40, new Rotation2d(90));
	private static final Pose2d RED_AMP_ALIGN_POSE = new Pose2d(14.72, 7.40, new Rotation2d(90));
	private static final Pose2d BLUE_AMP_SCORE_POSE = new Pose2d(1.82, 7.63, new Rotation2d(90));
	private static final Pose2d RED_AMP_SCORE_POSE = new Pose2d(14.72, 7.63, new Rotation2d(90));

	private static final double ACCELERATION_TOLERANCE = 0.1;

	private static final double COLLISION_DISTANCE = BUMPER_WIDTH_LENGTH + 0.3;

	public enum RobotState {
		IDLE {
			@Override
			public RobotState nextState() {
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
			public RobotState nextState() {
				// if the current command is null then this is the first time we're running this stage
				if (currentCommand == null) {
					input.s.launcherSubsystem.setAngle(LauncherSubsystem.RETRACTED_ANGLE);
					input.s.launcherSubsystem.stopLauncher();
					switch (input.goal) {
						case PICKUP_NOTE:
							currentPath =
									input.getPathToSource(input.s.drivebaseSubsystem.getPose().getTranslation());
							currentCommand = AutoBuilder.followPath(currentPath);
							currentCommand.schedule();
							break;
						case SCORE_SPEAKER:
							currentPath =
									input.getPathToSpeaker(input.s.drivebaseSubsystem.getPose().getTranslation());
							currentCommand = AutoBuilder.followPath(currentPath);
							currentCommand.schedule();
							break;
						case SCORE_AMP:
							currentPath =
									input.getPathToAmp(input.s.drivebaseSubsystem.getPose().getTranslation());
							currentCommand = AutoBuilder.followPath(currentPath);
							currentCommand.schedule();
							break;
						case PARK:
							currentPath =
									input.getPathToStage(input.s.drivebaseSubsystem.getPose().getTranslation());
							currentCommand = AutoBuilder.followPath(currentPath);
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
			public RobotState nextState() {
				input.avoidCollisionSpot();
				return TRAVELLING;
			}
		},
		SEARCHING {
			AllInCommand intakeCommand;
			boolean roamed;

			@Override
			public RobotState nextState() {
				if (intakeCommand == null) {
					Robot robot = Robot.getInstance();
					intakeCommand = new AllInCommand(input.s.intakeSubsystem, robot.controls);
					intakeCommand.schedule();
				}
				if (intakeCommand.isFinished()) {
					// we have intook
					intakeCommand = null;
					return TRAVELLING;
				}
				if (input.s.limelightSubsystem.hasTargets()
						&& currentCommand.isFinished()
						&& !currentCommand.getClass().equals(DriveToNoteCommand.class)) {
					currentCommand.cancel();
					currentCommand =
							new DriveToNoteCommand(input.s.drivebaseSubsystem, input.s.limelightSubsystem);
					currentCommand.schedule();
				} else if (currentCommand != null && currentCommand.isFinished()) {
					if (roamed) {}
					// currentCommand =
					// TODO: idk what is being done here jbear
				}
				return this;
			}
		},
		SPEAKER_SCORING {
			@Override
			public RobotState nextState() {
				if (currentCommand == null) {
					Robot robot = Robot.getInstance();
					currentCommand =
							new FullTargetCommand(
									input.s.launcherSubsystem, input.s.drivebaseSubsystem, robot.controls);
					currentCommand.schedule();
				}
				if (input.s.launcherSubsystem.isAtAngle() && input.s.launcherSubsystem.isAtSpeed()) {
					input.s.intakeSubsystem.feederShoot();
				}
				if (!input.s.intakeSubsystem.feederSensorHasNote()) {
					input.s.intakeSubsystem.feederStop();
					return IDLE;
				}
				return this;
			}
		},
		AMP_SCORING {
			@Override
			public RobotState nextState() {
				if (currentCommand == null) {
					currentCommand =
							new SetAngleAmpLaunchCommand(
									input.s.launcherSubsystem,
									LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
									LauncherSubsystem.AMP_AIM_ANGLE);
					currentCommand.schedule();
				}
				if (input.s.launcherSubsystem.isAtAngle()) {
					input.s.intakeSubsystem.feederShoot();
				}
				if (!input.s.intakeSubsystem.feederSensorHasNote()) {
					input.s.intakeSubsystem.feederStop();
					return IDLE;
				}
				return this;
			}
		};

		public abstract RobotState nextState();

		AutonomousTeleopSubsystem input;
		PathPlannerPath currentPath;
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

	private boolean enabled = false;
	private RobotState state;
	private RobotGoal goal;
	private ScoringMode scoringMode = ScoringMode.SPEAKER;
	private double matchTimeRemaining;
	private boolean inMatch = false;
	private Alliance alliance;

	// for collision detection
	private Translation2d lastExpectedVelocity;
	private double lastTimestamp;
	private Translation2d collisionDirection;

	public AutonomousTeleopSubsystem(Subsystems s) {
		this.s = s;

		// Pathfinder w/ obstacle avoidance
		Pathfinding.setPathfinder(new LocalADStar());
	}

	public void start() {
		enabled = true;
		inMatch = !(DriverStation.getMatchType().equals(MatchType.None));
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
		state.input = this;
		Pathfinding.ensureInitialized();
	}

	public void stop() {
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
			state = state.nextState();
		}
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
				s.drivebaseSubsystem.getSwerveDrive().getAccel().get().toTranslation2d();

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

	private PathPlannerPath getPathToSource(Translation2d robotPosition) {
		Pose2d sourcePose = alliance.equals(Alliance.Blue) ? BLUE_SOURCE_POSE : RED_SOURCE_POSE;
		return new PathPlannerPath(
				List.of(robotPosition, sourcePose.getTranslation()),
				CONSTRAINTS,
				new GoalEndState(0, sourcePose.getRotation()));
	}

	private PathPlannerPath getPathToSourceRoam(Translation2d robotPosition) {
		return null;
	}

	private PathPlannerPath getPathToSpeaker(Translation2d robotPosition) {
		return null;
	}

	private PathPlannerPath getPathToAmp(Translation2d robotPosition) {
		Pose2d ampPose = alliance.equals(Alliance.Blue) ? BLUE_AMP_ALIGN_POSE : RED_AMP_ALIGN_POSE;
		setGoalPosition(robotPosition, ampPose.getTranslation());

		return null;
	}

	private PathPlannerPath getPathToStage(Translation2d robotPosition) {
		return null;
	}

	public Command getPathFindingCommand(Pose2d goalPose) {
		// TODO: i think this is how were supposed to approach pathfinding?
		GoalEndState goalEndState = new GoalEndState(0, goalPose.getRotation());

		// TODO: think abt rotation delay parameter?
		return AutoBuilder.pathfindThenFollowPath(Pathfinding.getCurrentPath(CONSTRAINTS, goalEndState), CONSTRAINTS);
	}

	public Command scoreAmpCommand() {
		Pose2d ampScorePose = alliance.equals(Alliance.Blue) ? BLUE_AMP_SCORE_POSE : RED_AMP_SCORE_POSE;
		Pathfinding.setGoalPosition(ampScorePose.getTranslation());

		return Commands.sequence(
				new SetAngleAmpLaunchCommand(
						s.launcherSubsystem,
						LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
						LauncherSubsystem.AMP_AIM_ANGLE),
				getPathFindingCommand(ampScorePose),
				Commands.waitUntil(AutoLogic.isReadyToLaunch()).andThen(AutoLogic.feedUntilNoteLaunched()));
	}
}
