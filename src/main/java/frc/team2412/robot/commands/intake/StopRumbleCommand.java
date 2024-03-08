package frc.team2412.robot.commands.intake;
import frc.team2412.robot.Controls;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class StopRumbleCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
	private final Controls controls;

    public StopRumbleCommand(IntakeSubsystem intakeSubsystem, Controls controls){
        this.intakeSubsystem = intakeSubsystem;
		this.controls = controls;
    }
    
    
	@Override
	public void initialize() {
		controls.vibrateDriveController(0.0);
	}

    @Override
	public boolean isFinished() {
		return true;
	}

}
