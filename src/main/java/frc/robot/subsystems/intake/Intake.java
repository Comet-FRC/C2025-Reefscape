package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs;
	private final ArmVisualizer armVisualizer;

	public Intake(IntakeIO io) {
		this.io = io;
		this.inputs = new IntakeIOInputsAutoLogged();
		this.armVisualizer = new ArmVisualizer(getName(), IntakeConstants.LENGTH);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		armVisualizer.setArmAngle(inputs.pivotPosition);
		armVisualizer.publish();
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
		return Commands.runOnce(() -> io.setWheelVelocity(velocity), this);
	}
}
