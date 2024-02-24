package frc.team2412.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LEDSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LightsCommand extends Command {
    private final LEDSubsystem LEDSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LauncherSubsystem launcherSubsystem;

    public LightsCommand(
            LEDSubsystem LEDSubsystem, 
            IntakeSubsystem intakeSubsystem, 
            LauncherSubsystem launcherSubsystem) {
		this.LEDSubsystem = LEDSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.launcherSubsystem = launcherSubsystem;
		addRequirements(LEDSubsystem);
	}

    @Override
	public void initialize() {
		LEDSubsystem.setRED_LED();
	}

    @Override
    public void execute() {
        if (intakeSubsystem.getIndexSensor()) {
            LEDSubsystem.setBLUE_LED();
        } else if (launcherSubsystem.atTargetSpeed()) {
            LEDSubsystem.setGREEN_LED();
        }
    }

	@Override
	public boolean isFinished() {
        LEDSubsystem.setRED_LED();
		return true;
	}
}
