// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drive.SwerveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max
 * turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
    private final int moduleNumber;
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkMax driveSpark;
    private final SparkMax turnSpark;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANcoder turnCANcoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOSpark(int module) {
        this.moduleNumber = module;

        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();
        };
        driveSpark = new SparkMax(
                switch (module) {
                    case 0 -> FL_DRIVE_ID;
                    case 1 -> FR_DRIVE_ID;
                    case 2 -> BL_DRIVE_ID;
                    case 3 -> BR_DRIVE_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> FL_AZIMUTH_ID;
                    case 1 -> FR_AZIMUTH_ID;
                    case 2 -> BL_AZIMUTH_ID;
                    case 3 -> BR_AZIMUTH_ID;
                    default -> 0;
                },
                MotorType.kBrushless);

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();

        turnCANcoder = new CANcoder(
                switch (module) {
                    case 0 -> FL_CANCODER_ID;
                    case 1 -> FR_CANCODER_ID;
                    case 2 -> BL_CANCODER_ID;
                    case 3 -> BR_CANCODER_ID;
                    default -> 0;
                });

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new SparkMaxConfig();
        driveConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) DRIVE_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(12.0);
        driveConfig.encoder
                .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        DRIVE_kP, 0.0,
                        DRIVE_kD, 0.0);
        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.baseUnitMagnitude()))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(IS_AZIMUTH_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) AZIMUTH_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(12.0);
        turnConfig.encoder
                .positionConversionFactor(AZIMUTH_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(AZIMUTH_ENCODER_VELOCITY_FACTOR)
                .uvwAverageDepth(2);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .pidf(AZIMUTH_kP, 0.0, AZIMUTH_kD, 0.0);
        turnConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY.in(Hertz)))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(
                turnSpark,
                5,
                () -> turnEncoder.setPosition(
                        turnCANcoder
                                .getPosition().getValue()
                                .minus(zeroRotation.getMeasure()).in(Radians)));

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        Logger.recordOutput(
                "Drive/Module" + moduleNumber + "/cancoder measurement",
                turnCANcoder.getPosition().getValue().in(Radians));

        // Update drive inputs
        sparkStickyFault = false;
        ifOk(
                driveSpark,
                driveEncoder::getPosition,
                (value) -> inputs.drivePositionRad = Radians.of(value));
        ifOk(
                driveSpark,
                driveEncoder::getVelocity,
                (value) -> inputs.driveAngularVelocity = RadiansPerSecond.of(value));
        ifOk(
                driveSpark,
                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                (values) -> inputs.driveAppliedVolts = Volts.of(values[0] * values[1]));
        ifOk(
                driveSpark,
                driveSpark::getOutputCurrent,
                (value) -> inputs.driveCurrentAmps = Amps.of(value));
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;

        Logger.recordOutput("Angle Motor Position", turnEncoder.getPosition());
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value));
        ifOk(
                turnSpark,
                turnEncoder::getVelocity,
                (value) -> inputs.turnVelocityRadPerSec = RadiansPerSecond.of(value));

        ifOk(
                turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                (values) -> inputs.turnAppliedVolts = Volts.of(values[0] * values[1]));
        ifOk(
                turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = Amps.of(value));
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        // TODO: Check if this works
        if (this.turnEncoder.getVelocity() < 0.01) {
            tryUntilOk(
                    turnSpark,
                    5,
                    () -> turnEncoder.setPosition(
                            turnCANcoder
                                    .getPosition().getValue()
                                    .minus(zeroRotation.getMeasure()).in(Radians)));
        }
    }

    @Override
    public void setDriveOpenLoop(Voltage output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(Voltage output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        double velocityRadPerSec = velocity.in(RadiansPerSecond);
        double ffVolts = DRIVE_kS * Math.signum(velocityRadPerSec) + DRIVE_kV * velocityRadPerSec;
        driveController.setReference(
                velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(rotation.getRadians(), 0, 2 * Math.PI);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}
