package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Controls;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class AllInCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;
	private final Controls controls;
	private boolean rumbledIndex = false;

	public AllInCommand(IntakeSubsystem intakeSubsystem, Controls controls) {
		this.intakeSubsystem = intakeSubsystem;
		this.controls = controls;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.intakeIn();
		intakeSubsystem.indexIn();
		intakeSubsystem.feederIn();
	}

	@Override
	public void execute() {

		// all intake motors rejecting after index
		if (intakeSubsystem.indexSensorHasNote()) {
			if (!intakeSubsystem.getRejectOverride()) {
				intakeSubsystem.intakeReject();
			} else {
				intakeSubsystem.intakeStop();
			}

			if (controls != null && !rumbledIndex) {
				Commands.race(new RumbleCoDriveControllerCommand(controls), new WaitCommand(3)).schedule();
				rumbledIndex = true;
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (!intakeSubsystem.getRejectOverride()) {
			intakeSubsystem.intakeReject();
		} else {
			intakeSubsystem.intakeStop();
		}

		intakeSubsystem.indexStop();
		intakeSubsystem.feederStop();

		if (interrupted) {
			return;
		}

		if (controls != null) {
			Commands.race(new RumbleCoDriveControllerCommand(controls), new WaitCommand(1)).schedule();
			Commands.race(new RumbleDriveControllerCommand(controls), new WaitCommand(1)).schedule();
		}

		// rumbledIntakeFront = false;
		// rumbledIntakeLeft = false;
		// rumbledIntakeRight = false;
		rumbledIndex = false;
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.feederSensorHasNote();
	}
}
