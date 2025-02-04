package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.Angle;

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
					io.stop();
				},
			this);
	}

	public Command setLeftPosition(Angle position) {
		return Commands.run(() -> io.setLeftPosition(position), this);
	}

	public Command setRightPosition(Angle position) {
		return Commands.run(() -> io.setRightPosition(position), this);
	}
}
