// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakeRight;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.hoodtake.HoodtakeConstants;

import static edu.wpi.first.units.Units.*;


public class RightIntakeIOSim implements RightIntakeIO {
	private final DCMotorSim wheelMotor = configureWheelMotor();
	private final SingleJointedArmSim pivotMotor = configurePivotMotor();

	

	private static DCMotorSim configureWheelMotor() {
		DCMotor wheelGearbox = DCMotor.getNEO(1);
		LinearSystem<N2, N1, N2> wheelPlant = LinearSystemId.createDCMotorSystem(
			wheelGearbox,
			RightIntakeConstants.WHEEL_MOI,
			RightIntakeConstants.WHEEL_CONVERSION_FACTOR
		);
		return new DCMotorSim(wheelPlant, wheelGearbox);
	}

	private static SingleJointedArmSim configurePivotMotor() {
		return new SingleJointedArmSim(
			DCMotor.getNEO(1),
			RightIntakeConstants.PIVOT_CONVERSION_FACTOR,
			SingleJointedArmSim.estimateMOI(RightIntakeConstants.LENGTH.in(Meters), RightIntakeConstants.MASS.in(Kilograms)),
			RightIntakeConstants.LENGTH.in(Meters),
			-0.5,
			RightIntakeConstants.STARTING_ANGLE.in(Radians)+0.5,
			true,
			RightIntakeConstants.STARTING_ANGLE.in(Radians),
			RightIntakeConstants.PIVOT_ENCODER_DISTANCE_PER_PULSE,
			0
		);
	}

	private final PIDController wheelPID =
		new PIDController(
			RightIntakeConstants.WHEEL_SIM_kP,
			RightIntakeConstants.WHEEL_SIM_kI,
			RightIntakeConstants.WHEEL_SIM_kD
		);

	private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(
		RightIntakeConstants.WHEEL_SIM_kS,
		RightIntakeConstants.WHEEL_SIM_kV,
		RightIntakeConstants.WHEEL_SIM_kA
	);

	private final PIDController pivotPID = 
		new PIDController(
			RightIntakeConstants.PIVOT_SIM_kP,
			RightIntakeConstants.PIVOT_SIM_kI,
			RightIntakeConstants.PIVOT_SIM_kD
		);

	private final ArmFeedforward pivotFF =
		new ArmFeedforward(
			RightIntakeConstants.PIVOT_SIM_kS,
			RightIntakeConstants.PIVOT_SIM_kG,
			RightIntakeConstants.PIVOT_SIM_kV,
			RightIntakeConstants.PIVOT_SIM_kA
		);

	/** true = controlled by voltage, false = controled by PID + FF */
	private boolean pivotVoltageMode = false;
	private Voltage pivotAppliedVoltage = Volts.of(0.0);
	/** true = controlled by voltage, false = controled by PID + FF */
	private boolean wheelVoltageMode = false;
	
	public RightIntakeIOSim(){
		this.setPivotPositionSetpoint(HoodtakeConstants.STARTING_ANGLE);
	}

	@Override
	public void updateInputs(RightIntakeIOInputs inputs) {
		wheelMotor.update(0.02);
		pivotMotor.update(0.02);

		runLoopControl();

		inputs.wheelPosition = wheelMotor.getAngularPosition();
		inputs.wheelVelocity = wheelMotor.getAngularVelocity();
		inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getSetpoint());
		inputs.wheelAppliedVolts = Volts.of(wheelMotor.getInputVoltage());
		inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getCurrentDrawAmps());

		inputs.pivotPosition = Radians.of(pivotMotor.getAngleRads());
		inputs.pivotDesiredPosition = Radians.of(pivotPID.getSetpoint());
		inputs.pivotVelocity = RadiansPerSecond.of(pivotMotor.getVelocityRadPerSec());
		inputs.pivotAppliedVolts = this.pivotAppliedVoltage;
		inputs.pivotSupplyCurrent = Amps.of(pivotMotor.getCurrentDrawAmps());
	}

	private void runLoopControl() {
		if (!wheelVoltageMode) {
			wheelMotor.setInputVoltage(
				wheelPID.calculate(wheelMotor.getAngularVelocity().in(RadiansPerSecond))
				+
				wheelFF.calculate(wheelPID.getSetpoint())
			);
		}

		if (!pivotVoltageMode) {
			pivotMotor.setInputVoltage(
				pivotPID.calculate(pivotMotor.getAngleRads())
				+
				pivotFF.calculate(pivotPID.getSetpoint(), 0)
			);
		}
	}

	@Override
	public void setWheelVelocitySetpoint(AngularVelocity velocity) {
		this.wheelVoltageMode = false;
		wheelPID.setSetpoint(velocity.in(RadiansPerSecond));
	}

	@Override
	public void setWheelVoltage(Voltage volts) {
		this.wheelVoltageMode = true;
		wheelMotor.setInputVoltage(volts.in(Volts));
	}

	@Override
	public void stopWheel() {
		setWheelVoltage(Volts.of(0.0));
	}

	@Override
	public void setPivotPositionSetpoint(Angle position) {
		this.pivotVoltageMode = false;
		pivotPID.setSetpoint(position.in(Radians));
	}

	@Override
	public void setPivotVoltage(Voltage volts) {
		this.pivotVoltageMode = true;
		pivotMotor.setInputVoltage(volts.in(Volts));
		this.pivotAppliedVoltage = volts;
	}
	
	@Override
	public void stopPivot() {
		setPivotVoltage(Volts.of(0.0));
	}
}
