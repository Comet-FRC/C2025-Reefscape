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
			ShooterConstants.WHEEL_CONVERSION_FACTOR
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

	private final SimpleMotorFeedforward WheelFF = new SimpleMotorFeedforward(
		ShooterConstants.WHEEL_SIM_kS,
		ShooterConstants.WHEEL_SIM_kV,
		ShooterConstants.WHEEL_SIM_kA
	);

	/** true = controlled by voltage, false = controled by PID + FF */
	private boolean wheelVoltageMode = false;

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
		if (!wheelVoltageMode) {

			topWheelMotor.setInputVoltage(
				topPID.calculate(topWheelMotor.getAngularVelocity().in(RadiansPerSecond))
				+
				WheelFF.calculate(topPID.getSetpoint())
			);

			bottomWheelMotor.setInputVoltage(
				botPID.calculate(bottomWheelMotor.getAngularVelocity().in(RadiansPerSecond))
				+
				WheelFF.calculate(botPID.getSetpoint())
			);
		}


	}

	@Override
	public void setWheelVelocitySetpoint(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
		this.wheelVoltageMode = false;
		topPID.setSetpoint(topVelocity.in(RadiansPerSecond));
		botPID.setSetpoint(bottomVelocity.in(RadiansPerSecond));
	}

	@Override
	public void setWheelVoltage(Voltage volts) {
		this.wheelVoltageMode = true;
		topWheelMotor.setInputVoltage(volts.in(Volts));
		bottomWheelMotor.setInputVoltage(volts.in(Volts));
	}

	@Override
	public void stop() {
		setWheelVoltage(Volts.of(0.0));
	}

}
