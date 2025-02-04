// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.*;


public class IntakeIOSim implements IntakeIO {
	private final FlywheelSim wheelMotor = configureWheelMotor();
	private final SingleJointedArmSim pivotMotor = configurePivotMotor();

	private static FlywheelSim configureWheelMotor() {
		DCMotor wheelGearbox = DCMotor.getNEO(1);
		LinearSystem<N1, N1, N1> wheelPlant = LinearSystemId.createFlywheelSystem(
			wheelGearbox,
			IntakeConstants.WHEEL_CONVERSION_FACTOR,
			IntakeConstants.WHEEL_MOI
		);
		return new FlywheelSim(wheelPlant, wheelGearbox);
	}

	private static SingleJointedArmSim configurePivotMotor() {
		return new SingleJointedArmSim(
			DCMotor.getNEO(1),
			IntakeConstants.PIVOT_CONVERSION_FACTOR,
			SingleJointedArmSim.estimateMOI(IntakeConstants.LENGTH.in(Meters), IntakeConstants.MASS.in(Kilograms)),
			IntakeConstants.LENGTH.in(Meters),
			0.0,
			Math.PI,
			true,
			0,
			IntakeConstants.PIVOT_ENCODER_DISTANCE_PER_PULSE,
			0
		);
	}

	private final PIDController wheelPID =
		new PIDController(
			IntakeConstants.WHEEL_SIM_kP,
			IntakeConstants.WHEEL_SIM_kI,
			IntakeConstants.WHEEL_SIM_kD
		);

	private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(
		IntakeConstants.WHEEL_SIM_kS,
		IntakeConstants.WHEEL_SIM_kV,
		IntakeConstants.WHEEL_SIM_kA
	);

	private final PIDController pivotPID = 
		new PIDController(
			IntakeConstants.PIVOT_SIM_kP,
			IntakeConstants.PIVOT_SIM_kI,
			IntakeConstants.PIVOT_SIM_kD
		);

	private final ArmFeedforward pivotFF =
		new ArmFeedforward(
			IntakeConstants.PIVOT_SIM_kS,
			IntakeConstants.PIVOT_SIM_kG,
			IntakeConstants.PIVOT_SIM_kV,
			IntakeConstants.PIVOT_SIM_kA
		);

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
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
