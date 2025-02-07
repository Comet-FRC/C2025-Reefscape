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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOMapleSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(0.05, 0, 0);
  private PIDController turnController = new PIDController(8, 0, 0);
  private Voltage driveFFVolts = Volts.of(0.0);
  private Voltage driveAppliedVolts = Volts.of(0.0);
  private Voltage turnAppliedVolts = Volts.of(0.0);

  public ModuleIOMapleSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    // configures a generic motor controller for drive motor
    // set a current limit of 60 amps
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT);
    this.turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(SwerveConstants.AZIMUTH_CURRENT_LIMIT);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts.plus(
              Volts.of(
                  driveController.calculate(
                      moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond))));
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts =
          Volts.of(
              turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians()));
    } else {
      turnController.reset();
    }

    Voltage driveAppliedVoltage =
        Volts.of(MathUtil.clamp(driveAppliedVolts.in(Volts), -12.0, 12.0));
    Voltage turnAppliedVoltage = Volts.of(MathUtil.clamp(turnAppliedVolts.in(Volts), -12.0, 12.0));

    // Update simulation state
    driveMotor.requestVoltage(driveAppliedVoltage);
    turnMotor.requestVoltage(turnAppliedVoltage);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition();
    inputs.driveAngularVelocity = moduleSimulation.getDriveWheelFinalSpeed();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps =
        Amps.of(Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps)));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps =
        Amps.of(Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps)));

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    ;
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(Voltage output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(Voltage output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {
    double velocityRadPerSec = velocity.in(RadiansPerSecond);
    driveClosedLoop = true;
    driveFFVolts =
        Volts.of(
            SwerveConstants.DRIVE_SIM_kS * Math.signum(velocityRadPerSec)
                + SwerveConstants.DRIVE_SIM_kV * velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
