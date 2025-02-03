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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class DriveConstants {
  public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(12.5);
  public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(100.0); // Hz
  /** @see https://en.wikipedia.org/wiki/Axle_track */
  public static final Distance TRACK_WIDTH = Inches.of(19.5);
  /** @see https://en.wikipedia.org/wiki/Wheelbase */
  public static final Distance WHEELBASE = Inches.of(13.5);

  public static final Distance DRIVE_BASE_RADIUS =
      Meters.of(Math.hypot(TRACK_WIDTH.div(2.0).in(Meters), WHEELBASE.div(2.0).in(Meters)));
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(TRACK_WIDTH.div(2.0), WHEELBASE.div(2.0)),
        new Translation2d(TRACK_WIDTH.div(2.0), WHEELBASE.div(-2.0)),
        new Translation2d(TRACK_WIDTH.div(-2.0), WHEELBASE.div(2.0)),
        new Translation2d(TRACK_WIDTH.div(-2.0), WHEELBASE.div(-2.0))
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation =
      new Rotation2d(60.45132695667231 / (2 * Math.PI));
  public static final Rotation2d frontRightZeroRotation =
      new Rotation2d(48.32636373736527 / (2 * Math.PI));
  public static final Rotation2d backLeftZeroRotation =
      new Rotation2d(56.48999159646946 / (2 * Math.PI));
  public static final Rotation2d backRightZeroRotation =
      new Rotation2d(110.95594322787176 / (2 * Math.PI));

  /*public static final Rotation2d frontLeftZeroRotation = new Rotation2d();
  public static final Rotation2d frontRightZeroRotation = new Rotation2d();
  public static final Rotation2d backLeftZeroRotation = new Rotation2d();
  public static final Rotation2d backRightZeroRotation = new Rotation2d();*/

  //
  public static final boolean useHeadingCorrection = true;
  public static final boolean useCosineCompensation = true;

  // Device CAN IDs
  public static final int pigeonCanId = 20;

  public static final int frontLeftDriveCanId = 1;
  public static final int frontRightDriveCanId = 3;
  public static final int backLeftDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int frontRightTurnCanId = 4;
  public static final int backLeftTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftCancoderId = 9;
  public static final int frontRightCancoderId = 10;
  public static final int backLeftCancoderId = 11;
  public static final int backRightCancoderId = 12;

  // Drive motor configuration
  public static final Current driveMotorCurrentLimit = Amps.of(50);
  public static final Distance WHEEL_RADIUS = Inches.of(2);
  public static final double driveMotorReduction = (50.0 * 19.0 * 45.0) / (14.0 * 25.0 * 15.0);
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.14173;
  public static final double driveKv = 0.16171;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.12484;
  public static final double driveSimKv = 0.18236;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final Current turnMotorCurrentLimit = Amps.of(20);
  public static final double turnMotorReduction = 150.0 / 7.0;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  // TODO: look at this
  public static final Mass ROBOT_MASS = Pounds.of(86.5);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);
  /**
   * @see https://www.vexrobotics.com/colsonperforma.html
   */
  public static final double wheelCOF = 1.2;

  public static final RobotConfig ppConfig =
      new RobotConfig(
          ROBOT_MASS,
          ROBOT_MOI,
          new ModuleConfig(
              WHEEL_RADIUS,
              MAX_SPEED,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
