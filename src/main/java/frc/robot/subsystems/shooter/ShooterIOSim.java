// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;


import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

	

	

	private final FlywheelSim bottomMotorSim = setupBottomMotor();
	private final FlywheelSim topMotorSim = setupTopMotor();

	private static FlywheelSim setupBottomMotor() {
		DCMotor gearbox = DCMotor.getNEO(1);
		LinearSystem<N1, N1, N1> plant =
		LinearSystemId.createFlywheelSystem(gearbox, ShooterConstants.WHEEL_MOMENT_OF_INERTIA, ShooterConstants.WHEEL_CONVERSION_FACTOR);
		return new FlywheelSim(plant, gearbox);
	}

	private static FlywheelSim setupTopMotor() {
		DCMotor gearbox = DCMotor.getNEO(1);
		LinearSystem<N1, N1, N1> plant =
		LinearSystemId.createFlywheelSystem(gearbox, ShooterConstants.WHEEL_MOMENT_OF_INERTIA, ShooterConstants.WHEEL_CONVERSION_FACTOR);
		return new FlywheelSim(plant, gearbox);
	}

	private final PIDController bottomPID = new PIDController(ShooterConstants.SIM_kP, ShooterConstants.SIM_kI, ShooterConstants.SIM_kD);
	private final PIDController topPID = new PIDController(ShooterConstants.SIM_kP, ShooterConstants.SIM_kI, ShooterConstants.SIM_kD);
	private final SimpleMotorFeedforward bottomFF = new SimpleMotorFeedforward(ShooterConstants.SIM_kS, ShooterConstants.SIM_kV, ShooterConstants.SIM_kA);
	private final SimpleMotorFeedforward topFF = new SimpleMotorFeedforward(ShooterConstants.SIM_kS, ShooterConstants.SIM_kV, ShooterConstants.SIM_kA);

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		bottomMotorSim.update(0.02);
		topMotorSim.update(0.02);
		// control to setpoint

		this.setVoltage(
			Volts.of(bottomPID.calculate(bottomMotorSim.getAngularVelocity().in(RadiansPerSecond)) + bottomFF.calculate(bottomPID.getSetpoint())),
			Volts.of(topPID.calculate(bottomMotorSim.getAngularVelocity().in(RadiansPerSecond)) + topFF.calculate(topPID.getSetpoint()))
		);

		inputs.bottomVelocity = bottomMotorSim.getAngularVelocity();
		inputs.bottomDesiredVelocity = RadiansPerSecond.of(bottomPID.getSetpoint());
		inputs.bottomAppliedVoltage = Volts.of(bottomMotorSim.getInputVoltage());
		inputs.bottomSupplyCurrent = Amps.of(bottomMotorSim.getCurrentDrawAmps());
		
		inputs.topVelocity = topMotorSim.getAngularVelocity();
		inputs.topDesiredVelocity = RadiansPerSecond.of(topPID.getSetpoint());
		inputs.topAppliedVoltage = Volts.of(topMotorSim.getInputVoltage());
		inputs.topSupplyCurrent = Amps.of(topMotorSim.getCurrentDrawAmps());
	}

	@Override
	public void setVoltage(Voltage topVoltage, Voltage bottomVoltage) {
		topMotorSim.setInputVoltage(topVoltage.in(Volts));
		bottomMotorSim.setInputVoltage(bottomVoltage.in(Volts));
	}

	@Override
	public void setAngularVelocity(ShooterSpeed shooterSpeed) {
		this.topPID.setSetpoint(shooterSpeed.topMotorSpeed.in(RadiansPerSecond));
		this.bottomPID.setSetpoint(shooterSpeed.botMotorSpeed.in(RadiansPerSecond));
	}

	@Override
	public void setPID(double kP, double kI, double kD) {
		bottomPID.setPID(kP, kI, kD);
		topPID.setPID(kP, kI, kD);
	}

	@Override
	public void stop() {
		this.setAngularVelocity(new ShooterSpeed());
	}
}
