package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.util.LauncherDataLoader;
import frc.team2412.robot.util.LauncherDataPoint;

public class FullTargetCommand extends Command{

    private final Pose2d SPEAKER_POSE = new Pose2d(0.0, 5.55, Rotation2d.fromRotations(0));    // set later

    private Rotation2d yawAngle;
    private double angle;
    private double rpm;
    private Pose2d robotPose;
    private Pose2d relativeSpeaker;
    private double distance;
    private LauncherDataPoint dataPoint;
    private Command setAngleLaunchCommand;
    private Command rotateToAngle;

    DrivebaseSubsystem drivebaseSubsystem;
    LauncherSubsystem launcherSubsystem;
    public FullTargetCommand(LauncherSubsystem launcherSubsystem, DrivebaseSubsystem drivebaseSubsystem){
        this.launcherSubsystem = launcherSubsystem;
        this.drivebaseSubsystem = drivebaseSubsystem;
        setAngleLaunchCommand = new SetAngleLaunchCommand(launcherSubsystem, () -> rpm, () -> angle);
        rotateToAngle = drivebaseSubsystem.rotateToAngle(() -> yawAngle, true);
    }


    @Override
    public void initialize(){
        CommandScheduler.getInstance().schedule(setAngleLaunchCommand);
    }
    @Override
    public void execute(){
        robotPose = drivebaseSubsystem.getPose();
        relativeSpeaker = robotPose.relativeTo(SPEAKER_POSE);
        yawAngle = Rotation2d.fromRadians(Math.atan2(relativeSpeaker.getY(), relativeSpeaker.getX()));      
        distance = relativeSpeaker.getTranslation().getNorm();  
        dataPoint = LauncherDataLoader.fromCSV(new File(Filesystem.getDeployDirectory(), "launcher_data.csv")).get(distance);
        rpm = dataPoint.rpm;
        angle = dataPoint.angle;


    }

    @Override
    public void end(boolean interrupted){
        rotateToAngle.cancel();
        setAngleLaunchCommand.cancel();
    }
}
