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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public Angle drivePositionRad = Radians.of(0.0);
    public AngularVelocity driveAngularVelocity = RadiansPerSecond.of(0.0);
    public Voltage driveAppliedVolts = Volts.of(0.0);
    public Current driveCurrentAmps = Amps.of(0.0);

    public boolean turnConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public AngularVelocity turnVelocityRadPerSec = RadiansPerSecond.of(0.0);
    public Voltage turnAppliedVolts = Volts.of(0.0);
    public Current turnCurrentAmps = Amps.of(0.0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(Voltage output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(Voltage output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(AngularVelocity velocity) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
