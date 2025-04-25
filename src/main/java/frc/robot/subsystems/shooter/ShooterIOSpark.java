// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSpark implements ShooterIO, Runnable {
	private final LoggedTunableNumber TOP_WHEEL_kP = new LoggedTunableNumber("Shooter/Top Wheel kP", ShooterConstants.TOP_WHEEL_kP);
	private final LoggedTunableNumber TOP_WHEEL_kI = new LoggedTunableNumber("Shooter/Top Wheel kI", ShooterConstants.TOP_WHEEL_kI);
	private final LoggedTunableNumber TOP_WHEEL_kD = new LoggedTunableNumber("Shooter/Top Wheel kD", ShooterConstants.TOP_WHEEL_kD);
	private final LoggedTunableNumber BOT_WHEEL_kP = new LoggedTunableNumber("Shooter/Bot Wheel kP", ShooterConstants.BOT_WHEEL_kP);
	private final LoggedTunableNumber BOT_WHEEL_kI = new LoggedTunableNumber("Shooter/Bot Wheel kI", ShooterConstants.BOT_WHEEL_kI);
	private final LoggedTunableNumber BOT_WHEEL_kD = new LoggedTunableNumber("Shooter/Bot Wheel kD", ShooterConstants.BOT_WHEEL_kD);
	
	private final Notifier notifier = new Notifier(this::run);

	private final LoggedTunableNumber FLYWHEEL_ANGULAR_ACCELERATION = new LoggedTunableNumber("Shooter/Flywheel Angular Acceleration", 100);


	private final SparkMax topMotor = new SparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax botMotor = new SparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
	private final RelativeEncoder topEncoder = topMotor.getEncoder();
	private final RelativeEncoder botEncoder = botMotor.getEncoder();
	private final SparkClosedLoopController topPID = topMotor.getClosedLoopController();
	private final SparkClosedLoopController botPID = botMotor.getClosedLoopController();

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



	private final TrapezoidProfile topProfile = new TrapezoidProfile(
		/*
		 * Since we're using velocity control, maxVelocity is really
		 * angular acceleration, and maxAcceleration is really
		 * jerk.
		 */
		new TrapezoidProfile.Constraints(250, 2400)
	);

	private final TrapezoidProfile bottomProfile = new TrapezoidProfile(
		/*
		 * Since we're using velocity control, maxVelocity is really
		 * angular acceleration, and maxAcceleration is really
		 * jerk.
		 */
		new TrapezoidProfile.Constraints(400, 3800)
	);

	public ShooterIOSpark() {
		this.configureTopMotor();
		this.configureBottomMotor();
		SparkUtil.tryUntilOk(
			topMotor,
			5,
			() -> topEncoder.setPosition(
				0
			)
		);
		SparkUtil.tryUntilOk(
			botMotor,
			5,
			() -> botEncoder.setPosition(
				0
			)
		);

		notifier.startPeriodic(0.02);
	}

	private void configureTopMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(true)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.TOP_WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.TOP_WHEEL_CONVERSION_FACTOR / 60.0)
			.uvwAverageDepth(2)
			.uvwMeasurementPeriod(10);
		config.signals
			.primaryEncoderPositionAlwaysOn(true)
			.primaryEncoderPositionPeriodMs(20)
			.primaryEncoderVelocityAlwaysOn(true)
			.primaryEncoderVelocityPeriodMs(20)
			.appliedOutputPeriodMs(20)
			.busVoltagePeriodMs(20)
			.outputCurrentPeriodMs(20);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(TOP_WHEEL_kP.get())
				.i(TOP_WHEEL_kI.get())
				.d(TOP_WHEEL_kD.get());

		SparkUtil.tryUntilOk(
			topMotor, 5,
			() -> topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
		);
	}

	private void configureBottomMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(false)
			.idleMode(IdleMode.kCoast)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.BOTTOM_WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.BOTTOM_WHEEL_CONVERSION_FACTOR / 60.0)
			.uvwAverageDepth(2)
			.uvwMeasurementPeriod(10);
		config.signals
			.primaryEncoderPositionAlwaysOn(true)
			.primaryEncoderPositionPeriodMs(20)
			.primaryEncoderVelocityAlwaysOn(true)
			.primaryEncoderVelocityPeriodMs(20)
			.appliedOutputPeriodMs(20)
			.busVoltagePeriodMs(20)
			.outputCurrentPeriodMs(20);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(BOT_WHEEL_kP.get())
				.i(BOT_WHEEL_kI.get())
				.d(BOT_WHEEL_kD.get());

		SparkUtil.tryUntilOk(
			botMotor, 5,
			() -> botMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
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
		inputs.topWheelPosition = Radians.of(topMotor.getEncoder().getPosition());
		inputs.topWheelVelocity = RadiansPerSecond.of(topMotor.getEncoder().getVelocity());
		inputs.topWheelDesiredVelocity = this.topWheelDesiredVelocity.copy();
		inputs.topWheelAppliedVoltage = Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
		inputs.topWheelSupplyCurrent = Amps.of(topMotor.getOutputCurrent());
		inputs.topTemperature = Celsius.of(topMotor.getMotorTemperature());
		
		inputs.bottomWheelPosition = Radians.of(botMotor.getEncoder().getPosition());
		inputs.bottomWheelVelocity = RadiansPerSecond.of(botMotor.getEncoder().getVelocity());
		inputs.bottomWheelDesiredVelocity = this.bottomWheelDesiredVelocity.copy();
		inputs.bottomWheelAppliedVoltage = Volts.of(botMotor.getAppliedOutput() * botMotor.getBusVoltage());
		inputs.bottomWheelSupplyCurrent = Amps.of(botMotor.getOutputCurrent());
		inputs.bottomWheelTemperature= Celsius.of(botMotor.getMotorTemperature());
	}

	@Override
	public void setTopVelocitySetpoint(AngularVelocity velocity) {
		this.topVoltageMode = false;
		this.topWheelDesiredVelocity.mut_replace(velocity);
		this.configureTopMotor();
	}

	@Override
	public void setBottomVelocitySetpoint(AngularVelocity velocity) {
		this.bottomVoltageMode = false;
		this.bottomWheelDesiredVelocity.mut_replace(velocity);
		this.configureBottomMotor();
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

	@Override
	public void run() {
		if (topVoltageMode) {
			this.topMotor.setVoltage(topDesiredVoltage.in(Volts));
		} else {
			TrapezoidProfile.State currentState =
				new TrapezoidProfile.State(
					topMotor.getEncoder().getVelocity(),
					0
				);

			TrapezoidProfile.State goalState =
				new TrapezoidProfile.State(
					topWheelDesiredVelocity.in(RadiansPerSecond),
					0
				);
			
			TrapezoidProfile.State velocitySetpoint = topProfile.calculate(0.02, currentState, goalState);
			
			this.topMotor.getClosedLoopController().setReference(
				velocitySetpoint.position,
				ControlType.kVelocity,
				ClosedLoopSlot.kSlot0,
				topWheelFF.calculate(velocitySetpoint.position, velocitySetpoint.velocity),
				ArbFFUnits.kVoltage
			);
		}

		if (bottomVoltageMode) {
			this.botMotor.setVoltage(bottomDesiredVoltage.in(Volts));
		} else {
			TrapezoidProfile.State currentState =
				new TrapezoidProfile.State(
					botEncoder.getVelocity(),
					0
				);

			TrapezoidProfile.State goalState =
				new TrapezoidProfile.State(
					bottomWheelDesiredVelocity.in(RadiansPerSecond),
					0
				);
			
			TrapezoidProfile.State velocitySetpoint = bottomProfile.calculate(0.02, currentState, goalState);
			
			this.botPID.setReference(
				velocitySetpoint.position,
				ControlType.kVelocity,
				ClosedLoopSlot.kSlot0,
				bottomWheelFF.calculate(velocitySetpoint.position, velocitySetpoint.velocity),
				ArbFFUnits.kVoltage
			);
		}
	}
}
