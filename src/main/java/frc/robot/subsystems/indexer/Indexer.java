package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmVisualizer3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
	private final IndexerIO io;
	private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
	private final ArmVisualizer3d leftArmVisualizer3d;
	private final ArmVisualizer3d rightArmVisualizer;


	public Indexer(IndexerIO io) {
		this.io = io;
		this.leftArmVisualizer3d = new ArmVisualizer3d("leftIndexer", new Translation3d(0,0,0), Rotation2d.fromDegrees(0));
		this.rightArmVisualizer = new ArmVisualizer3d("rightIndexer", new Translation3d(0,0,0), Rotation2d.fromDegrees(0));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Indexer", inputs);

		leftArmVisualizer3d.setArmAngle(inputs.leftPosition);
		leftArmVisualizer3d.publish();

		rightArmVisualizer.setArmAngle(inputs.rightPosition);
		rightArmVisualizer.publish();
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
		return Commands.run(() -> io.setLeftPositionSetpoint(position), this);
	}

	public Command setRightPosition(Angle position) {
		return Commands.run(() -> io.setRightPositionSetpoint(position), this);
	}
}
