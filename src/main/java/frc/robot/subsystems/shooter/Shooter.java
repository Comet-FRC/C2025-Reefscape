package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	public final ShooterIO io;
	public final ShooterIOInputsAutoLogged inputs;
	private final RangeTable RANGE_TABLE = new RangeTable();

	public Shooter(ShooterIO ShooterIO) {
		this.io = ShooterIO;
		this.inputs = new ShooterIOInputsAutoLogged();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);
	}


	public boolean readyToShoot(){
		if (inputs.topDesiredVelocity.minus(inputs.topVelocity).abs(RPM) > ShooterConstants.ACCEPTABLE_VELOCITY_ERROR.in(RPM))
			return false;
		if (inputs.bottomDesiredVelocity.minus(inputs.bottomVelocity).abs(RPM) > ShooterConstants.ACCEPTABLE_VELOCITY_ERROR.in(RPM))
			return false;
		return true;
	}

	public Command stop() {
		return Commands.run(() -> io.setAngularVelocity(new ShooterSpeed(RPM.of(0), RPM.of(0))));
	}

		//TODO: Lowk bad practice, figure out cleaner way
	public Command shootFromDistance(Supplier<Distance> distance) {
		return Commands.run(() -> {
				AngularVelocity topSpeed = RANGE_TABLE.get(distance.get().in(Meters)).getTopMotorSpeed();
				AngularVelocity botSpeed = RANGE_TABLE.get(distance.get().in(Meters)).getBotMotorSpeed();
				io.setAngularVelocity(new ShooterSpeed(topSpeed, botSpeed));
				
		});
	}

	public Command shoot(AngularVelocity topSpeed, AngularVelocity botSpeed) {
		return Commands.run(() -> {
				io.setAngularVelocity(new ShooterSpeed(topSpeed, botSpeed));
		});
	}

	public AngularVelocity getTopVelocity(){
		return inputs.topVelocity;
	}
	public AngularVelocity getBottomVelocity(){
		return inputs.bottomVelocity;
	}
}

