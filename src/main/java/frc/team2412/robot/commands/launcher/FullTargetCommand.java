package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.geometry.Rotation2d;

import com.fasterxml.jackson.databind.deser.std.ArrayBlockingQueueDeserializer;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class FullTargetCommand {

    private final Pose2d SPEAKER_POSE = 0.0;    // set later

    private Rotation2d yawAngle;
    private double rpm;
    private Pose2d robotPose;
    private Pose2d relativeSpeaker;

    DrivebaseSubsystem drivebaseSubsystem;
    LauncherSubsystem launcherSubsystem;
    public FullTargetCommand(){
        robotPose = drivebaseSubsystem.getPose();
        relativeSpeaker = robotPose.relativeTo(SPEAKER_POSE);
        yawAngle = Rotation2d.fromRadians(Math.atan2(relativeSpeaker.getY(), relativeSpeaker.getX()));        

    }
}
