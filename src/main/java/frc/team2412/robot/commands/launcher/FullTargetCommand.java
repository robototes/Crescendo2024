package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class FullTargetCommand {

    private final Pose2d SPEAKER_POSE = 0.0;    

    private double angle;
    private double rpm;
    private Pose2d robotPose;

    DrivebaseSubsystem drivebaseSubsystem;
    LauncherSubsystem launcherSubsystem;
    public FullTargetCommand(){
        robotPose = drivebaseSubsystem.getPose();


    }
}
