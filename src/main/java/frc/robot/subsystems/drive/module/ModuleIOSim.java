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
import static frc.robot.subsystems.drive.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = true;
  private boolean turnClosedLoop = true;
  private PIDController driveController = new PIDController(DRIVE_SIM_kP, 0, DRIVE_SIM_kD);
  private PIDController turnController = new PIDController(AZIMUTH_SIM_kP, 0, AZIMUTH_SIM_kD);
  private Voltage driveFFVolts = Volts.of(0.0);
  private Voltage driveAppliedVolts = Volts.of(0.0);
  private Voltage turnAppliedVolts = Volts.of(0.0);

  public ModuleIOSim() {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, DRIVE_MOTOR_REDUCTION),
            DCMotor.getNEO(1));
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, AZIMUTH_REDUCTION),
            DCMotor.getNEO(1));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts.plus(
              Volts.of(driveController.calculate(driveSim.getAngularVelocityRadPerSec())));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = Volts.of(turnController.calculate(turnSim.getAngularPositionRad()));
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts.in(Volts), -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts.in(Volts), -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = Radians.of(driveSim.getAngularPositionRad());
    inputs.driveAngularVelocity =
        RadiansPerSecond.of(
            driveSim.getAngularVelocityRadPerSec() + 1 * (Math.random() - Math.random()));
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad.in(Radians)};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
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
        Volts.of(DRIVE_SIM_kS * Math.signum(velocityRadPerSec) + DRIVE_SIM_kV * velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
