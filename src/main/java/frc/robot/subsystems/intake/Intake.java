package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	public final IntakeIO io;
	public final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	public Intake(IntakeIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
	}

	private Command stop() {
		return
			Commands.run(
				() -> {
					io.stopPivot();
					io.stopWheel();
				},
			this);
	}

	public Command setPosition(Angle position) {
		return Commands.run(() -> io.setPivotPosition(position), this);
	}

	public Command setWheelVelocity(AngularVelocity velocity) {
		return Commands.run(() -> io.setWheelVelocity(velocity), this);
	}
}
