package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
	public final IndexerIO io;
	public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

	public Indexer(IndexerIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Indexer", inputs);
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
