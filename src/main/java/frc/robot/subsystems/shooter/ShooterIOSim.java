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


	private final PIDController topWheelPID =
		new PIDController(
			ShooterConstants.topWheelSIM_kP,
			ShooterConstants.topWheelSIM_kI,
			ShooterConstants.topWheelSIM_kD
		);

	private final SimpleMotorFeedforward topWheelFF = new SimpleMotorFeedforward(
		ShooterConstants.topWheelSIM_kS,
		ShooterConstants.topWheelSIM_kV,
		ShooterConstants.topWheelSIM_kA
	);

	private final PIDController bottomWheelPID = 
		new PIDController(
			ShooterConstants.bottomWheelSIM_kP,
			ShooterConstants.bottomWheelSIM_kI,
			ShooterConstants.bottomWheelSIM_kD
		);

	private final SimpleMotorFeedforward bottomWheelFF =
		new SimpleMotorFeedforward(
			ShooterConstants.bottomWheelSIM_kS,
			ShooterConstants.bottomWheelSIM_kV,
			ShooterConstants.bottomWheelSIM_kA
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
		inputs.topWheelDesiredVelocity = RadiansPerSecond.of(topWheelPID.getSetpoint());
		inputs.topWheelAppliedVoltage = Volts.of(topWheelMotor.getInputVoltage());
		inputs.topWheelSupplyCurrent = Amps.of(topWheelMotor.getCurrentDrawAmps());

		inputs.bottomWheelPosition = topWheelMotor.getAngularPosition();
		inputs.bottomWheelVelocity = topWheelMotor.getAngularVelocity();
		inputs.bottomWheelDesiredVelocity = RadiansPerSecond.of(bottomWheelPID.getSetpoint());
		inputs.bottomWheelAppliedVoltage = Volts.of(topWheelMotor.getInputVoltage());
		inputs.bottomWheelSupplyCurrent = Amps.of(topWheelMotor.getCurrentDrawAmps());
	
	}

	private void runLoopControl() {
		if (!wheelVoltageMode) {
			topWheelMotor.setInputVoltage(
				topWheelPID.calculate(topWheelMotor.getAngularVelocity().in(RadiansPerSecond))
				+
				topWheelFF.calculate(topWheelPID.getSetpoint())
			);

			bottomWheelMotor.setInputVoltage(
				bottomWheelPID.calculate(bottomWheelMotor.getAngularVelocity().in(RadiansPerSecond))
				+
				bottomWheelFF.calculate(bottomWheelPID.getSetpoint())
			);
		}


	}

	@Override
	public void setWheelVelocitySetpoint(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
		this.wheelVoltageMode = false;
		topWheelPID.setSetpoint(topVelocity.in(RadiansPerSecond));
		bottomWheelPID.setSetpoint(bottomVelocity.in(RadiansPerSecond));
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
