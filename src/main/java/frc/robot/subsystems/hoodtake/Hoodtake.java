package frc.robot.subsystems.hoodtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Hoodtake extends SubsystemBase {
	public final HoodtakeIO io;
	public final HoodtakeIOInputsAutoLogged inputs = new HoodtakeIOInputsAutoLogged();

	public Hoodtake(HoodtakeIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Hoodtake", inputs);
	}

	public Command stop() {
		return Commands.run(
				() -> {
					io.stopPivot();
					io.stopWheel();
				},
				this);
	}

	public Command setPosition(Supplier<Angle> position) {
		return Commands.run(() -> io.setPivotPositionSetpoint(position.get()), this);
	}

	public Command setWheelVelocity(Supplier<AngularVelocity> velocity) {
		return Commands.run(() -> io.setWheelVelocitySetpoint(velocity.get()), this);
	}
}
