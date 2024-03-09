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
			intakeSubsystem.intakeBackReject();
			intakeSubsystem.intakeLeftReject();
			intakeSubsystem.intakeRightReject();

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(0.5)).schedule();
			}
		}

		if (intakeSubsystem.intakeBackSeesNote()) {
			intakeSubsystem.intakeFrontReject();
			intakeSubsystem.intakeLeftReject();
			intakeSubsystem.intakeRightReject();

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(0.5)).schedule();
			}
		}

		if (intakeSubsystem.intakeLeftSeesNote()) {
			intakeSubsystem.intakeFrontReject();
			intakeSubsystem.intakeBackReject();
			intakeSubsystem.intakeRightReject();

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(0.5)).schedule();
			}
		}

		if (intakeSubsystem.intakeRightSeesNote()) {
			intakeSubsystem.intakeFrontReject();
			intakeSubsystem.intakeBackReject();
			intakeSubsystem.intakeLeftReject();

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(0.5)).schedule();
			}
		}

		// all intake motors rejecting after index
		if (intakeSubsystem.indexSensorHasNote()) {
			intakeSubsystem.intakeReject();

			if (controls != null) {
				Commands.race(new RumbleCommand(controls), new WaitCommand(1)).schedule();
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.intakeReject();
		intakeSubsystem.indexStop();
		intakeSubsystem.feederStop();

		if (controls != null) {
			Commands.race(new RumbleCommand(controls), new WaitCommand(1)).schedule();
		}
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.feederSensorHasNote();
	}
}
