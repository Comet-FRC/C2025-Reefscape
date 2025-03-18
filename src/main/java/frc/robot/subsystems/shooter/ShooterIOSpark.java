// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSpark implements ShooterIO {
	private final LoggedTunableNumber FLYWHEEL_ANGULAR_ACCELERATION = new LoggedTunableNumber("Shooter/Flywheel Angular Acceleration", 100);


	private final SparkMax topMotor = new SparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax bottomMotor = new SparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

		private final SimpleMotorFeedforward topWheelFF = new SimpleMotorFeedforward(
		ShooterConstants.TOP_WHEEL_kS,
		ShooterConstants.TOP_WHEEL_kV,
		ShooterConstants.TOP_WHEEL_kA
	);
	private final SimpleMotorFeedforward bottomWheelFF = new SimpleMotorFeedforward(
		ShooterConstants.BOT_WHEEL_kS,
		ShooterConstants.BOT_WHEEL_kV,
		ShooterConstants.BOT_WHEEL_kA
	);

	public ShooterIOSpark() {
		this.configureTopMotor();
		this.configureBottomMotor();
	}

	private void configureTopMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(true)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.TOP_WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.TOP_WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(ShooterConstants.TOP_WHEEL_kP)
				.i(ShooterConstants.TOP_WHEEL_kI)
				.d(ShooterConstants.TOP_WHEEL_kD)
				.velocityFF(1.0/473.0 * (2 * Math.PI / 60))
			.maxMotion
				.maxAcceleration(FLYWHEEL_ANGULAR_ACCELERATION.get());

		topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
		SparkUtil.tryUntilOk(
			topMotor,
			5,
			() -> topMotor.getEncoder().setPosition(
				0
			)
		);
	}
	private void configureBottomMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(false)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.BOTTOM_WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.BOTTOM_WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(ShooterConstants.BOT_WHEEL_kP)
				.i(ShooterConstants.BOT_WHEEL_kI)
				.d(ShooterConstants.BOT_WHEEL_kD)
				.velocityFF(1.0/473.0 * (2 * Math.PI / 60))
			.maxMotion
				.maxAcceleration(FLYWHEEL_ANGULAR_ACCELERATION.get());

		bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
		SparkUtil.tryUntilOk(
			bottomMotor,
			5,
			() -> bottomMotor.getEncoder().setPosition(
				0
			)
		);
	}

	MutAngularVelocity topWheelDesiredVelocity = RadiansPerSecond.mutable(0);
	MutAngularVelocity bottomWheelDesiredVelocity = RadiansPerSecond.mutable(0);

	MutVoltage topDesiredVoltage = Volts.mutable(0);
	boolean topVoltageMode = false;
	
	MutVoltage bottomDesiredVoltage = Volts.mutable(0);
	boolean bottomVoltageMode = false;

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		if (topVoltageMode) {
			this.topMotor.setVoltage(topDesiredVoltage.in(Volts));
		}

		if (bottomVoltageMode) {
			this.bottomMotor.setVoltage(bottomDesiredVoltage.in(Volts));
		}

		inputs.topWheelPosition = Radians.of(topMotor.getEncoder().getPosition());
		inputs.topWheelVelocity = RadiansPerSecond.of(topMotor.getEncoder().getVelocity());
		inputs.topWheelAppliedVoltage = Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
		inputs.topWheelSupplyCurrent = Amps.of(topMotor.getOutputCurrent());
		inputs.topTemperature = Celsius.of(topMotor.getMotorTemperature());
		inputs.topWheelDesiredVelocity = this.topWheelDesiredVelocity.copy();
		
		inputs.bottomWheelPosition = Radians.of(bottomMotor.getEncoder().getPosition());
		inputs.bottomWheelVelocity = RadiansPerSecond.of(bottomMotor.getEncoder().getVelocity());
		inputs.bottomWheelAppliedVoltage = Volts.of(bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage());
		inputs.bottomWheelSupplyCurrent = Amps.of(bottomMotor.getOutputCurrent());
		inputs.bottomWheelTemperature= Celsius.of(bottomMotor.getMotorTemperature());
		inputs.bottomWheelDesiredVelocity = this.bottomWheelDesiredVelocity.copy();
	}

	@Override
	public void setBottomVelocitySetpoint(AngularVelocity velocity) {
		this.bottomVoltageMode = false;

		double bottomVelocityRadiansPerSecond = velocity.in(RadiansPerSecond);

		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(false)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.BOTTOM_WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.BOTTOM_WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(SmartDashboard.getNumber("Shooter/botP", 0))
				.i(SmartDashboard.getNumber("Shooter/botI", 0))
				.d(SmartDashboard.getNumber("Shooter/botD", 0))
				.velocityFF(1.0/473.0 * (2 * Math.PI / 60))
			.maxMotion
				.maxAcceleration(FLYWHEEL_ANGULAR_ACCELERATION.get());

		bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		//double bottomFeedForward = bottomWheelFF.calculate(bottomVelocityRadiansPerSecond);


		bottomMotor.getClosedLoopController().setReference(
			bottomVelocityRadiansPerSecond,
			ControlType.kMAXMotionVelocityControl
		);

		this.bottomWheelDesiredVelocity.mut_replace(velocity);	
	}

	@Override
	public void setTopVelocitySetpoint(AngularVelocity velocity) {
		this.topVoltageMode = false;
		double topVelocityRadiansPerSecond = velocity.in(RadiansPerSecond);
		//double topFeedForward = topWheelFF.calculate(topVelocityRadiansPerSecond);

		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(true)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.TOP_WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.TOP_WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(SmartDashboard.getNumber("Shooter/topP", 0))
				.i(SmartDashboard.getNumber("Shooter/topI", 0))
				.d(SmartDashboard.getNumber("Shooter/topD", 0))
				.velocityFF(1.0/473.0 * (2 * Math.PI / 60))
			.maxMotion
				.maxAcceleration(FLYWHEEL_ANGULAR_ACCELERATION.get());
		topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		// System.out.println(FLYWHEEL_ANGULAR_ACCELERATION.get());

		topMotor.getClosedLoopController().setReference(
			topVelocityRadiansPerSecond,
			ControlType.kMAXMotionVelocityControl
		);

		// System.out.println("topD: " + topMotor.configAccessor.closedLoop.getD());



		this.topWheelDesiredVelocity.mut_replace(velocity);
	}

	@Override
	public void stop() {
		this.setWheelVelocitySetpoint(RadiansPerSecond.of(0), RadiansPerSecond.of(0));
	}

	@Override
	public void setTopVoltage(Voltage volts) {
		this.topVoltageMode = true;
		this.topDesiredVoltage.mut_replace(volts);
	}

	@Override
	public void setBottomVoltage(Voltage volts) {
		this.bottomVoltageMode = true;
		this.bottomDesiredVoltage.mut_replace(volts);
	}
}
