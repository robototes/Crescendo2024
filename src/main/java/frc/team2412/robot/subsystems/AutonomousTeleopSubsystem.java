package frc.team2412.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Subsystems;

public class AutonomousTeleopSubsystem extends SubsystemBase {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(3., 3., 3., 3.);

    private static final double PARK_TIME = 15.;
    private static final double FORCE_PARK_TIME = 5.;

    private static final Translation2d BLUE_STAGE_POSITION = new Translation2d(4.88, 4.05);
    private static final Translation2d RED_STAGE_POSITION = new Translation2d(11.7, 4.05);
    private static final double STAGE_SIZE = 1.2;
    
    // TODO: get these points
    private static final Pose2d BLUE_SOURCE_POSE = new Pose2d();
    private static final Pose2d RED_SOURCE_POSE = new Pose2d();

    private static final double ACCELERATION_TOLERANCE = 0.1;

    public enum RobotState {
        IDLE {
            @Override
            public RobotState nextState() {
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
                return TRAVELLING;
            }
        },
        TRAVELLING {
            @Override
            public RobotState nextState() {
                // if the current command is null then this is the first time we're running this stage
                if (currentCommand == null) {
                    input.s.launcherSubsystem.setAngle(LauncherSubsystem.RETRACTED_ANGLE);
                    switch (input.goal) {
                        case PICKUP_NOTE:
                            currentPath = input.getPathToSource(input.s.drivebaseSubsystem.getPose().getTranslation());
                            currentCommand = AutoBuilder.followPath(currentPath);
                            currentCommand.schedule();
                            break;
                        case SCORE_SPEAKER:
                            currentPath = input.getPathToSpeaker(input.s.drivebaseSubsystem.getPose().getTranslation());
                            currentCommand = AutoBuilder.followPath(currentPath);
                            currentCommand.schedule();
                            break;
                        case SCORE_AMP:
                            currentPath = input.getPathToAmp(input.s.drivebaseSubsystem.getPose().getTranslation());
                            currentCommand = AutoBuilder.followPath(currentPath);
                            currentCommand.schedule();
                            break;
                        case PARK:
                            currentPath = input.getPathToStage(input.s.drivebaseSubsystem.getPose().getTranslation());
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
            @Override
            public RobotState nextState() {
                return null;
            }
        },
        SPEAKER_SCORING {
            @Override
            public RobotState nextState() {
                return null;
            }
        },
        AMP_SCORING {
            @Override
            public RobotState nextState() {
                return null;
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
    private ScoringMode scoringMode;
    private double matchTimeRemaining;
    private boolean inMatch = false;
    private Alliance alliance;

    // for collision detection
    private double lastExpectedVelocity;
    private double lastTimestamp;

    public AutonomousTeleopSubsystem(Subsystems s) {
        this.s = s;
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
        return inMatch && matchTimeRemaining > PARK_TIME;
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
        double expectedVelocity = new Translation2d(s.drivebaseSubsystem.getFieldSpeeds().vxMetersPerSecond, s.drivebaseSubsystem.getFieldSpeeds().vyMetersPerSecond).getNorm();
        // estimate expected acceleration using a = Δv/Δt
        double expectedAccel = (lastExpectedVelocity - expectedVelocity) / (lastTimestamp - Timer.getFPGATimestamp());
        double actualAccel = s.drivebaseSubsystem.getSwerveDrive().getAccel().get().toTranslation2d().getNorm();

        double accelDifference = Math.abs(expectedAccel - actualAccel);

        lastExpectedVelocity = expectedVelocity;
        lastTimestamp = Timer.getFPGATimestamp();

        if (Math.abs(accelDifference) > ACCELERATION_TOLERANCE) {
            return true;
        }
        return false;
    }

    public void avoidCollisionSpot() {

    }

    private PathPlannerPath getPathToSource(Translation2d robotPosition) {
        Pose2d sourcePose = alliance.equals(Alliance.Blue) ? BLUE_SOURCE_POSE : RED_SOURCE_POSE;
        return new PathPlannerPath(List.of(robotPosition, sourcePose.getTranslation()), CONSTRAINTS, new GoalEndState(0, sourcePose.getRotation()));
    }

    private PathPlannerPath getPathToSpeaker(Translation2d robotPosition) {
        return null;
    }

    private PathPlannerPath getPathToAmp(Translation2d robotPosition) {
        return null;
    }

    private PathPlannerPath getPathToStage(Translation2d robotPosition) {
        return null;
    }
}
