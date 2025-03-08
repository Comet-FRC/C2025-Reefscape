// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static edu.wpi.first.units.Units.*;


public class ShooterIOSim implements ShooterIO {
	private final DCMotorSim topWheelMotor = configureWheelMotor();
	private final DCMotorSim bottomWheelMotor = configureWheelMotor();

	private static DCMotorSim configureWheelMotor() {
		DCMotor wheelGearbox = DCMotor.getNEO(1);
		LinearSystem<N2, N1, N2> wheelPlant = LinearSystemId.createDCMotorSystem(
			wheelGearbox,
			ShooterConstants.WHEEL_MOMENT_OF_INERTIA,
			ShooterConstants.TOP_WHEEL_CONVERSION_FACTOR / (2.0 * Math.PI) 
		);
		return new DCMotorSim(wheelPlant, wheelGearbox);
	}


	private final PIDController topPID =
		new PIDController(
			ShooterConstants.WHEEL_SIM_kP,
			ShooterConstants.WHEEL_SIM_kI,
			ShooterConstants.WHEEL_SIM_kD
		);
	private final PIDController botPID =
		new PIDController(
			ShooterConstants.WHEEL_SIM_kP,
			ShooterConstants.WHEEL_SIM_kI,
			ShooterConstants.WHEEL_SIM_kD
		);

	private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(
		ShooterConstants.WHEEL_SIM_kS,
		ShooterConstants.WHEEL_SIM_kV,
		ShooterConstants.WHEEL_SIM_kA
	);

	private boolean topVoltageMode = false;
	private boolean bottomVoltageMode = false;

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		topWheelMotor.update(0.02);
		bottomWheelMotor.update(0.02);

		runLoopControl();

		inputs.topWheelPosition = topWheelMotor.getAngularPosition();
		inputs.topWheelVelocity = topWheelMotor.getAngularVelocity();
		inputs.topWheelDesiredVelocity = RadiansPerSecond.of(topPID.getSetpoint());
		inputs.topWheelAppliedVoltage = Volts.of(topWheelMotor.getInputVoltage());
		inputs.topWheelSupplyCurrent = Amps.of(topWheelMotor.getCurrentDrawAmps());

		inputs.bottomWheelPosition = topWheelMotor.getAngularPosition();
		inputs.bottomWheelVelocity = topWheelMotor.getAngularVelocity();
		inputs.bottomWheelDesiredVelocity = RadiansPerSecond.of(topPID.getSetpoint());
		inputs.bottomWheelAppliedVoltage = Volts.of(topWheelMotor.getInputVoltage());
		inputs.bottomWheelSupplyCurrent = Amps.of(topWheelMotor.getCurrentDrawAmps());
	
	}

	private void runLoopControl() {
		if (!topVoltageMode) {

			topWheelMotor.setInputVoltage(
				topPID.calculate(topWheelMotor.getAngularVelocity().in(RadiansPerSecond))
				// +
				// wheelFF.calculate(topPID.getSetpoint())
			);
		}

		if (!bottomVoltageMode) {
			bottomWheelMotor.setInputVoltage(
				botPID.calculate(bottomWheelMotor.getAngularVelocity().in(RadiansPerSecond))
				// +
				// wheelFF.calculate(botPID.getSetpoint())
			);
		}
	}

	@Override
	public void setWheelVelocitySetpoint(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
		this.topVoltageMode = false;
		this.bottomVoltageMode = false;
		this.topPID.setSetpoint(topVelocity.in(RadiansPerSecond));
		this.botPID.setSetpoint(bottomVelocity.in(RadiansPerSecond));
	}

	@Override
	public void setTopVoltage(Voltage volts) {
		this.topVoltageMode = true;
		this.topWheelMotor.setInputVoltage(volts.in(Volts));
	}

	@Override
	public void setBottomVoltage(Voltage volts) {
		this.topVoltageMode = true;
		this.bottomWheelMotor.setInputVoltage(volts.in(Volts));
	}


	@Override
	public void stop() {
		this.setTopVoltage(Volts.of(0.0));
		this.setBottomVoltage(Volts.of(0.0));
	}

}
