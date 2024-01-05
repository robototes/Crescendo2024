package frc.team2412.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DrivebaseSubsystem extends SubsystemBase {

    // SWERVE CONSTANTS (that aren't in deploy dir)

    private static final double MAX_SPEED = 1.0;

    private final SwerveDrive swerveDrive;

    public DrivebaseSubsystem() {
        File swerveJsonDirectory;

        if (Robot.getInstance().isCompetition()) {
            swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        } else {
            swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "practiceswerve");
        }

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException();
        }

        // set drive motors to brake
        swerveDrive.setMotorIdleMode(true);
        // enable optimization (never move the angle wheels more than 90 degrees)
        swerveDrive.setModuleStateOptimization(true);
        // swerve drive heading will slowly drift over time as you translate. this method enables an active correction using pid. disabled until testing can be done
        swerveDrive.setHeadingCorrection(false);
        // supposed to do something? see https://broncbotz3481.github.io/YAGSL/swervelib/SwerveDrive.html#chassisVelocityCorrection
        swerveDrive.chassisVelocityCorrection = true;

        swerveDrive.synchronizeModuleEncoders();
    }

    public void drive(
        Translation2d translation,
        Rotation2d rotation,
        boolean fieldOriented) {
        swerveDrive.drive(translation, rotation.getRadians(), fieldOriented, false);
    }

    /*
     * Set the robot's pose.
     * TODO: does this change yaw too? does this affect field oriented?
     */
    public void setPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /*
     * Reset the gyro angle. After this method is called, yaw will be zero. Pose is also updated to zero rotation but same position
     */
    public void resetGyro() {
        swerveDrive.zeroGyro();
    }

    /*
     * Reset everything we can on the drivebase. To be used before auto starts
     */
    public void resetRobot() {
        swerveDrive.resetEncoders();
        resetGyro();
        setPose(new Pose2d());
    }
}
