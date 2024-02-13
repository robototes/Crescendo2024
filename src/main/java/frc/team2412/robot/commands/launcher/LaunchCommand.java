package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class LaunchCommand extends Command {
    private final LauncherSubsystem launcherSubsystem;
    private final IntakeSubsystem IntakeSubsystem;

    public LaunchCommand(LauncherSubsystem launcherSubsystem, IntakeSubsystem IntakeSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.IntakeSubsystem = IntakeSubsystem;
        addRequirements(launcherSubsystem, IntakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.feederOut();
        launcherSubsystem.launch;
    }

    @Override
    public boolean isFinished() {
        return launcherSubsystem.areFlywheelsAtTargetSpeed();
    }

    @Override
    public void end() {
        while (!launcherSubsystem.areFlywheelsAtTargetSpeed()) {
        }
        launcherSubsystem.launchBall();
        IntakeSubsystem.stop();
        launcherSubsystem.stopFlywheels();
    }
}