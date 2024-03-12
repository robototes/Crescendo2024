package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Controls;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class AllInCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;
	private final Controls controls;

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
		// intake rejecting
		if (intakeSubsystem.intakeFrontSeesNote()) {
			if (!intakeSubsystem.getRejectOverride()) {
				intakeSubsystem.intakeBackReject();
				intakeSubsystem.intakeLeftReject();
				intakeSubsystem.intakeRightReject();
			} else {
				intakeSubsystem.intakeBackStop();
				intakeSubsystem.intakeLeftStop();
				intakeSubsystem.intakeRightStop();
			}

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(1)).schedule();
			}
		}

		if (intakeSubsystem.intakeBackSeesNote()) {
			if (!intakeSubsystem.getRejectOverride()) {
				intakeSubsystem.intakeFrontReject();
				intakeSubsystem.intakeLeftReject();
				intakeSubsystem.intakeRightReject();
			} else {
				intakeSubsystem.intakeFrontStop();
				intakeSubsystem.intakeLeftStop();
				intakeSubsystem.intakeRightStop();
			}

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(2)).schedule();
			}
		}

		if (intakeSubsystem.intakeLeftSeesNote()) {
			if (!intakeSubsystem.getRejectOverride()) {
				intakeSubsystem.intakeFrontReject();
				intakeSubsystem.intakeBackReject();
				intakeSubsystem.intakeRightReject();
			} else {
				intakeSubsystem.intakeFrontStop();
				intakeSubsystem.intakeBackStop();
				intakeSubsystem.intakeRightStop();
			}

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(2)).schedule();
			}
		}

		if (intakeSubsystem.intakeRightSeesNote()) {
			if (!intakeSubsystem.getRejectOverride()) {
				intakeSubsystem.intakeFrontReject();
				intakeSubsystem.intakeBackReject();
				intakeSubsystem.intakeLeftReject();
			} else {
				intakeSubsystem.intakeFrontStop();
				intakeSubsystem.intakeBackStop();
				intakeSubsystem.intakeLeftStop();
			}

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(2)).schedule();
			}
		}

		// all intake motors rejecting after index
		if (intakeSubsystem.indexSensorHasNote()) {
			if (!intakeSubsystem.getRejectOverride()) {
				intakeSubsystem.intakeReject();
			} else {
				intakeSubsystem.intakeStop();
			}

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(3)).schedule();
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

		if (controls != null) {
			Commands.race(new RumbleCommand(controls), new WaitCommand(3)).schedule();
		}
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.feederSensorHasNote();
	}
}
