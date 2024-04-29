package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.LAUNCHER_ENABLED;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.drivebase.DriveToNoteCommand;
import frc.team2412.robot.commands.intake.AllInCommand;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.launcher.AimTowardsSpeakerCommand;
import frc.team2412.robot.commands.launcher.PrepFlywheelForLaunchCommand;
import frc.team2412.robot.commands.launcher.SetAngleAmpLaunchCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.commands.launcher.StopLauncherCommand;
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
	private static final Translation2d RED_STAGE_POSITION = new Translation2d(11.69, 4.09);
	private static final double STAGE_SIZE = 1.2;

	private static final double BLUE_WING_LINE = 5.85;
	private static final double RED_WING_LINE = 10.70;

	private static final Pose2d BLUE_SOURCE_POSE =
			new Pose2d(13.09, 0.86, new Rotation2d(-17.45 + 180));
	private static final Pose2d RED_SOURCE_POSE =
			new Pose2d(3.09, 0.86, Rotation2d.fromDegrees(-170.79));
	private static final PathPlannerPath BLUE_SOURCE_ROAM_PATH =
			PathPlannerPath.fromPathFile("BlueSourceRoamPath");
	private static final PathPlannerPath RED_SOURCE_ROAM_PATH =
			PathPlannerPath.fromPathFile("RedSourceRoamPath");

	// These points are the center of the speaker
	private static final Translation2d BLUE_SPEAKER_POSITION = new Translation2d(0.05, 5.53);
	private static final Translation2d RED_SPEAKER_POSITION = new Translation2d(16.55, 5.53);
	// Distance we can safely launch into the speaker from in meters
	private static final double SPEAKER_LAUNCHING_DISTANCE = 2.5;

	// amp
	private static final Pose2d BLUE_AMP_ALIGN_POSE =
			new Pose2d(1.82, 7.40, Rotation2d.fromDegrees(-90));
	private static final Pose2d RED_AMP_ALIGN_POSE =
			new Pose2d(14.72, 7.40, Rotation2d.fromDegrees(-90));
	private static final Pose2d BLUE_AMP_SCORE_POSE =
			new Pose2d(1.82, 7.63, Rotation2d.fromDegrees(-90));
	private static final Pose2d RED_AMP_SCORE_POSE =
			new Pose2d(14.72, 7.63, Rotation2d.fromDegrees(-90));

	private static final double AMP_PIVOTING_DISTANCE_TOLERANCE = 2.5;

	// trap poses
	private static final double TRAP_ALIGNMENT_DISTANCE = 1.83; // TODO: test and tune
	private static final double TRAP_SCORING_DISTANCE = 0.8; // TODO: test and tune

	// TODO: verify calculations, i think i did it correctly but the math has given me aneurysms to
	// the point of me not knowing anymore
	public static final Pose2d[] BLUE_TRAP_ALIGNMENT_POSES = {
		// CENTERLINE SIDE
		new Pose2d(
				BLUE_STAGE_POSITION.getX() + TRAP_SCORING_DISTANCE,
				BLUE_STAGE_POSITION.getY(),
				new Rotation2d(180.0)),
		// SOURCE SIDE
		new Pose2d(
				BLUE_STAGE_POSITION.getX()
						- (TRAP_ALIGNMENT_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				BLUE_STAGE_POSITION.getY()
						- (TRAP_ALIGNMENT_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(60.0)),
		// AMP SIDE
		new Pose2d(
				BLUE_STAGE_POSITION.getX()
						- (TRAP_ALIGNMENT_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				BLUE_STAGE_POSITION.getY()
						+ (TRAP_ALIGNMENT_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(-60.0))
	};

	public static final Pose2d[] RED_TRAP_ALIGNMENT_POSES = {
		// CENTERLINE SIDE
		new Pose2d(
				RED_STAGE_POSITION.getX() - TRAP_SCORING_DISTANCE,
				RED_STAGE_POSITION.getY(),
				new Rotation2d(0.0)),
		// SOURCE SIDE
		new Pose2d(
				RED_STAGE_POSITION.getX()
						+ (TRAP_ALIGNMENT_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				RED_STAGE_POSITION.getY()
						- (TRAP_ALIGNMENT_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(120.0)),
		// AMP SIDE
		new Pose2d(
				RED_STAGE_POSITION.getX()
						+ (TRAP_ALIGNMENT_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				RED_STAGE_POSITION.getY()
						+ (TRAP_ALIGNMENT_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(-120.0))
	};

	public static final Pose2d[] BLUE_TRAP_SCORING_POSES = {
		// CENTERLINE SIDE
		new Pose2d(
				BLUE_STAGE_POSITION.getX() + TRAP_SCORING_DISTANCE,
				BLUE_STAGE_POSITION.getY(),
				new Rotation2d(180.0)),
		// SOURCE SIDE
		new Pose2d(
				BLUE_STAGE_POSITION.getX() - (TRAP_SCORING_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				BLUE_STAGE_POSITION.getY() - (TRAP_SCORING_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(60.0)),
		// AMP SIDE
		new Pose2d(
				BLUE_STAGE_POSITION.getX() - (TRAP_SCORING_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				BLUE_STAGE_POSITION.getY() + (TRAP_SCORING_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(-60.0))
	};

	public static final Pose2d[] RED_TRAP_SCORING_POSES = {
		// CENTERLINE SIDE
		new Pose2d(
				RED_STAGE_POSITION.getX() - TRAP_SCORING_DISTANCE,
				RED_STAGE_POSITION.getY(),
				new Rotation2d(0.0)),
		// SOURCE SIDE
		new Pose2d(
				RED_STAGE_POSITION.getX() + (TRAP_SCORING_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				RED_STAGE_POSITION.getY() - (TRAP_SCORING_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(120.0)),
		// AMP SIDE
		new Pose2d(
				RED_STAGE_POSITION.getX() + (TRAP_SCORING_DISTANCE * Math.sin(Units.degreesToRadians(30))),
				RED_STAGE_POSITION.getY() + (TRAP_SCORING_DISTANCE * Math.cos(Units.degreesToRadians(30))),
				new Rotation2d(-120.0))
	};

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
									: input.s.drivebaseSubsystem.getPose().getX() > RED_WING_LINE) {
								input.revFlyWheels().schedule();
							}

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
				currentCommand = null;
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

	private SendableChooser<ScoringMode> scoringModeChooser;
	private SendableChooser<RobotState> forceState;
	private SendableChooser<RobotGoal> forceGoal;

	// for collision detection
	private Translation2d lastExpectedVelocity = new Translation2d();
	private double lastTimestamp = 0;
	private Translation2d collisionDirection = new Translation2d();

	private Command pathfindToStageCommand;

	public AutonomousTeleopSubsystem(Subsystems s) {
		this.s = s;

		// Pathfinder w/ obstacle avoidance
		Pathfinding.setPathfinder(new LocalADStar());
		PathfindingCommand.warmupCommand().withName("PathplannerWarmupCommand").schedule();

		initData();
	}

	public void start() {
		enabled = true;
		inMatch = !(DriverStation.getMatchType().equals(MatchType.None));
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
		state = RobotState.IDLE;
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
			if (matchTimeRemaining <= FORCE_PARK_TIME && inMatch) {
				if (pathfindToStageCommand == null || pathfindToStageCommand.isFinished()) {
					pathfindToStageCommand = pathfindToStage();
					pathfindToStageCommand.schedule();
				}
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
					if (state.currentCommand != null) return state.currentCommand.getName();
					else return "None";
				});
		tab.addDouble("Match Time", () -> matchTimeRemaining);
		tab.addBoolean("Is Match", () -> inMatch);
		tab.addBoolean("Has Note", this::hasNote);
		tab.addBoolean("In Midgame", this::inMidgame);
		tab.addBoolean("Under Stage", this::underStage);
		tab.addBoolean("Is Colliding", this::isColliding);
		tab.addString("Alliance", () -> alliance.toString());

		scoringModeChooser = new SendableChooser<ScoringMode>();
		scoringModeChooser.setDefaultOption("Speaker", ScoringMode.SPEAKER);
		scoringModeChooser.addOption("Amp", ScoringMode.AMP);
		scoringModeChooser.onChange((ScoringMode mode) -> setScoringMode(mode));
		tab.add("Scoring Mode", scoringModeChooser);

		forceState = new SendableChooser<RobotState>();
		forceState.setDefaultOption("None", null);
		for (RobotState state : RobotState.values()) {
			forceState.addOption(state.name(), state);
		}
		tab.add("Force State", forceState);

		forceGoal = new SendableChooser<RobotGoal>();
		forceGoal.setDefaultOption("None", null);
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

	private boolean isNearPosition(Translation2d expectedPosition, double tolerance) {
		return (MathUtil.isNear(
						expectedPosition.getX(), s.drivebaseSubsystem.getPose().getX(), tolerance)
				&& MathUtil.isNear(
						expectedPosition.getY(), s.drivebaseSubsystem.getPose().getY(), tolerance));
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
		Translation2d stagePosition =
				alliance.equals(Alliance.Blue) ? BLUE_STAGE_POSITION : RED_STAGE_POSITION;
		Pose2d stagePose = new Pose2d(stagePosition, s.drivebaseSubsystem.getPose().getRotation());
		return pathfindToPose(stagePose);
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
		PathPlannerPath roamPath =
				alliance.equals(Alliance.Blue) ? BLUE_SOURCE_ROAM_PATH : RED_SOURCE_ROAM_PATH;
		roamPath.preventFlipping = true;
		return Commands.race(
						AutoBuilder.followPath(roamPath)
								.repeatedly()
								.until(s.limelightSubsystem::hasTargets)
								.andThen(new DriveToNoteCommand(s.drivebaseSubsystem, s.limelightSubsystem)),
						intake())
				.withName("SearchNoteCommand");
	}

	public Command trapCommand() {
		Pose2d alignmentPose =
				s.drivebaseSubsystem
						.getPose()
						.nearest(
								List.of(
										alliance.equals(Alliance.Blue)
												? BLUE_TRAP_ALIGNMENT_POSES
												: RED_TRAP_ALIGNMENT_POSES));
		Pose2d launchingPose =
				s.drivebaseSubsystem
						.getPose()
						.nearest(
								List.of(
										alliance.equals(Alliance.Blue)
												? BLUE_TRAP_SCORING_POSES
												: RED_TRAP_SCORING_POSES));

		return retractPivot()
				.andThen(pathfindToPose(alignmentPose))
				.andThen(prepTrapLaunchCommand())
				.andThen(pathfindToPose(launchingPose))
				.andThen(launch())
				.withName("ScoreTrapCommand");
	}

	// SUBSYSTEM COMMANDS

	public Command scoreAmpCommand() {
		Pose2d ampAlignPose = alliance.equals(Alliance.Blue) ? BLUE_AMP_ALIGN_POSE : RED_AMP_ALIGN_POSE;
		Pose2d ampScorePose = alliance.equals(Alliance.Blue) ? BLUE_AMP_SCORE_POSE : RED_AMP_SCORE_POSE;
		Pathfinding.setGoalPosition(ampScorePose.getTranslation());

		return retractPivot()
				.andThen(
						Commands.parallel(
										pathfindToPose(ampAlignPose).andThen(pathfindToPose(ampScorePose)),
										Commands.waitUntil(
														() ->
																isNearPosition(
																		ampScorePose.getTranslation(), AMP_PIVOTING_DISTANCE_TOLERANCE))
												.andThen(prepAmpLaunchCommand()))
								.andThen(launch()))
				.withName("ScoreAmpCommand");
	}

	public Command scoreSpeaker() {
		return (LAUNCHER_ENABLED
						? new AimTowardsSpeakerCommand(s.launcherSubsystem, s.drivebaseSubsystem)
								.andThen(launch())
						: Commands.none())
				.withName("ScoreSpeakerCommand");
	}

	@SuppressWarnings("unused")
	public Command launch() {
		return (LAUNCHER_ENABLED && INTAKE_ENABLED
						? Commands.waitUntil(AutoLogic.isReadyToLaunch())
								.andThen(
										new FeederInCommand(s.intakeSubsystem).until(AutoLogic.untilFeederHasNoNote()))
								.andThen(new StopLauncherCommand(s.launcherSubsystem))
						: Commands.none())
				.withName("LaunchCommand");
	}

	public Command intake() {
		return (INTAKE_ENABLED
						? new SetAngleLaunchCommand(s.launcherSubsystem, 0, LauncherSubsystem.RETRACTED_ANGLE)
								.andThen(new AllInCommand(s.intakeSubsystem, null))
						: Commands.none())
				.withName("IntakeCommand");
	}

	public Command revFlyWheels() {
		return (LAUNCHER_ENABLED
						? new PrepFlywheelForLaunchCommand(s.launcherSubsystem, s.drivebaseSubsystem)
						: Commands.none())
				.withName("RevFlywheelsCommand");
	}

	public Command retractPivot() {
		return (LAUNCHER_ENABLED
						? new InstantCommand(
								() -> s.launcherSubsystem.setAngle(LauncherSubsystem.RETRACTED_ANGLE))
						: Commands.none())
				.withName("RetractPivotCommand");
	}

	public Command prepAmpLaunchCommand() {
		return (LAUNCHER_ENABLED
						? new SetAngleAmpLaunchCommand(
								s.launcherSubsystem,
								LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
								LauncherSubsystem.AMP_AIM_ANGLE)
						: Commands.none())
				.withName("PrepAmpLaunchCommand");
	}

	public Command prepTrapLaunchCommand() {
		return (LAUNCHER_ENABLED
						? new SetAngleLaunchCommand(
								s.launcherSubsystem,
								LauncherSubsystem.TRAP_SHOOT_SPEED_RPM,
								LauncherSubsystem.TRAP_AIM_ANGLE)
						: Commands.none())
				.withName("PrepTrapLaunchCommandCommand");
	}
}
