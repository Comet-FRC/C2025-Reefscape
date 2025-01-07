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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private PIDController turnController = new PIDController(turnSimP, 0, turnSimD);
  private Voltage driveFFVolts = Volts.of(0.0);
  private Voltage driveAppliedVolts = Volts.of(0.0);
  private Voltage turnAppliedVolts = Volts.of(0.0);

  public ModuleIOSim() {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

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
        Volts.of(driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
