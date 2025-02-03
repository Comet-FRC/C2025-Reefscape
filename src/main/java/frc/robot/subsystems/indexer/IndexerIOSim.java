// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.*;


public class IndexerIOSim implements IndexerIO {
	private final FlywheelSim wheelMotor = configureWheelMotor();
	private final SingleJointedArmSim pivotMotor = configurePivotMotor();

	private static FlywheelSim configureWheelMotor() {
		DCMotor wheelGearbox = DCMotor.getNEO(1);
		LinearSystem<N1, N1, N1> wheelPlant = LinearSystemId.createFlywheelSystem(
			wheelGearbox,
			IndexerConstants.WHEEL_CONVERSION_FACTOR,
			IndexerConstants.WHEEL_MOI
		);
		return new FlywheelSim(wheelPlant, wheelGearbox);
	}

	private static SingleJointedArmSim configurePivotMotor() {
		return new SingleJointedArmSim(
			DCMotor.getNEO(1),
			IndexerConstants.PIVOT_CONVERSION_FACTOR,
			SingleJointedArmSim.estimateMOI(IndexerConstants.LENGTH.in(Meters), IndexerConstants.MASS.in(Kilograms)),
			IndexerConstants.LENGTH.in(Meters),
			0.0,
			Math.PI,
			true,
			0,
			IndexerConstants.PIVOT_ENCODER_DISTANCE_PER_PULSE,
			0
		);
	}

	private final PIDController wheelPID =
		new PIDController(
			IndexerConstants.WHEEL_SIM_kP,
			IndexerConstants.WHEEL_SIM_kI,
			IndexerConstants.WHEEL_SIM_kD
		);

	private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(
		IndexerConstants.WHEEL_SIM_kS,
		IndexerConstants.WHEEL_SIM_kV,
		IndexerConstants.WHEEL_SIM_kA
	);

	private final PIDController pivotPID = 
		new PIDController(
			IndexerConstants.PIVOT_SIM_kP,
			IndexerConstants.PIVOT_SIM_kI,
			IndexerConstants.PIVOT_SIM_kD
		);

	private final ArmFeedforward pivotFF =
		new ArmFeedforward(
			IndexerConstants.PIVOT_SIM_kS,
			IndexerConstants.PIVOT_SIM_kG,
			IndexerConstants.PIVOT_SIM_kV,
			IndexerConstants.PIVOT_SIM_kA
		);

	@Override
	public void updateInputs(IndexerIOInputs inputs) {
		wheelMotor.update(0.02);
		pivotMotor.update(0.02);

		runLoopControl();

		inputs.wheelVelocity = wheelMotor.getAngularVelocity();
		inputs.wheelAppliedVolts = Volts.of(wheelMotor.getInputVoltage());
		inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getCurrentDrawAmps());

		inputs.pivotPosition = Radians.of(pivotMotor.getAngleRads());
		inputs.pivotVelocity = RadiansPerSecond.of(pivotMotor.getVelocityRadPerSec());
		inputs.pivotSupplyCurrent = Amps.of(pivotMotor.getCurrentDrawAmps());
	}

	private void runLoopControl() {
		setWheelVoltage(
			Volts.of(
				wheelPID.calculate(wheelMotor.getAngularVelocity().in(RadiansPerSecond))
				+
				wheelFF.calculate(wheelPID.getSetpoint())
			)
		);

		setPivotVoltage(
			Volts.of(
				pivotPID.calculate(pivotMotor.getAngleRads())
				+
				pivotFF.calculate(pivotPID.getSetpoint(), 0)
			)
		);
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		wheelPID.setSetpoint(velocity.in(RadiansPerSecond));
	}

	@Override
	public void setWheelVoltage(Voltage volts) {
		wheelMotor.setInputVoltage(volts.in(Volts));
	}

	@Override
	public void setWheelPID(double kP, double kI, double kD) {
		wheelPID.setPID(kP, kI, kD);
	}

	@Override
	public void stopWheel() {
		wheelPID.setSetpoint(0);
		setWheelVoltage(Volts.of(0.0));
	}

	@Override
	public void setPivotPosition(Angle position) {
		pivotPID.setSetpoint(position.in(Radians));
	}

	@Override
	public void setPivotVoltage(Voltage volts) {
		pivotMotor.setInputVoltage(volts.in(Volts));
	}

	@Override
	public void runCharacterizationWheel(double input) {
		setWheelVoltage(Volts.of(input));
	}
}
