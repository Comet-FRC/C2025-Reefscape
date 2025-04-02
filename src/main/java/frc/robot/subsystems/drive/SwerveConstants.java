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

public class SwerveConstants {
    public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(12.5);
    public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(100.0); // Hz
    
    /** @see https://en.wikipedia.org/wiki/Axle_track */
    public static final Distance TRACK_WIDTH = Inches.of(23.25);
    /** @see https://en.wikipedia.org/wiki/Wheelbase */
    public static final Distance WHEELBASE = Inches.of(23.25);

    public static final Distance DRIVE_BASE_RADIUS = Meters.of(Math.hypot(TRACK_WIDTH.div(2.0).in(Meters), WHEELBASE.div(2.0).in(Meters)));
    
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(TRACK_WIDTH.div(2.0), WHEELBASE.div(2.0)),
            new Translation2d(TRACK_WIDTH.div(2.0), WHEELBASE.div(-2.0)),
            new Translation2d(TRACK_WIDTH.div(-2.0), WHEELBASE.div(2.0)),
            new Translation2d(TRACK_WIDTH.div(-2.0), WHEELBASE.div(-2.0))
    };

    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Radians.of(-4.517573356628418));
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(Radians.of(0.2070874124765396));
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Radians.of(3.4591267108917236));
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Radians.of(-7.534914016723633));

    // Device CAN IDs
    public static final int pigeonCanId = 21;

    public static final int FL_DRIVE_ID = 1;
    public static final int FR_DRIVE_ID = 3;
    public static final int BL_DRIVE_ID = 5;
    public static final int BR_DRIVE_ID = 7;

    public static final int FL_AZIMUTH_ID = 2;
    public static final int FR_AZIMUTH_ID = 4;
    public static final int BL_AZIMUTH_ID = 6;
    public static final int BR_AZIMUTH_ID = 8;

    public static final int FL_CANCODER_ID = 9;
    public static final int FR_CANCODER_ID = 10;
    public static final int BL_CANCODER_ID = 11;
    public static final int BR_CANCODER_ID = 12;

    // Drive motor configuration
    public static final Distance WHEEL_RADIUS = Inches.of(2);
    public static final Current DRIVE_CURRENT_LIMIT = Amps.of(60); // I got this value from pathplanner
    public static final double DRIVE_MOTOR_REDUCTION = (50.0 * 19.0 * 45.0) / (14.0 * 25.0 * 15.0);

    // Drive encoder configuration
    public static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_MOTOR_REDUCTION; // Rotor Rotations -> Wheel Radians
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / DRIVE_MOTOR_REDUCTION; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double DRIVE_kP = 0.01;
    public static final double DRIVE_kD = 0.0;
    public static final double DRIVE_kS = 0.06674;
    public static final double DRIVE_kV = 0.15844;
    public static final double DRIVE_kA = 0.022025;
    public static final double DRIVE_SIM_kP = 0.05;
    public static final double DRIVE_SIM_kD = 0.0;
    public static final double DRIVE_SIM_kS = 0.02493;
    public static final double DRIVE_SIM_kV = 0.18470;

    // Turn motor configuration
    public static final boolean IS_AZIMUTH_INVERTED = true;
    public static final Current AZIMUTH_CURRENT_LIMIT = Amps.of(20);
    public static final double AZIMUTH_REDUCTION = 150.0 / 7.0;

    // Turn encoder configuration
    public static final double AZIMUTH_ENCODER_POSITION_FACTOR = 2 * Math.PI / AZIMUTH_REDUCTION; // Rotations -> Radians
    public static final double AZIMUTH_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / AZIMUTH_REDUCTION; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double AZIMUTH_kP = 0.9;
    public static final double AZIMUTH_kD = 0.0;
    public static final double AZIMUTH_SIM_kP = 8.0;
    public static final double AZIMUTH_SIM_kD = 0.0;

    // PathPlanner configuration
    public static final Mass ROBOT_MASS = Pounds.of(105.3);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(8.7149124654810442678774120317821);
    /** @see https://www.vexrobotics.com/colsonperforma.html */
    public static final double wheelCOF = 1.2;

    public static final RobotConfig PATHPLANNER_CONFIG = new RobotConfig(
            ROBOT_MASS,
            ROBOT_MOI,
            new ModuleConfig(
                    WHEEL_RADIUS,
                    MAX_SPEED,
                    wheelCOF,
                    DCMotor.getNEO(1).withReduction(DRIVE_MOTOR_REDUCTION),
                    DRIVE_CURRENT_LIMIT,
                    1),
            MODULE_TRANSLATIONS);
}
