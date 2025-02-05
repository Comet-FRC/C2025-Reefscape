package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;

import static edu.wpi.first.units.Units.*;


public class ShootOnMove extends Command {
    private static final double G = 9.81; // Gravity (m/s²)
    private static final double SHOOTER_ANGLE = Math.toRadians(35); // Fixed angle
    private static final double MAX_RPM = 5000; // Max shooter RPM
    private static final double MAX_SHOT_SPEED = 15.0; // Max exit velocity (m/s)

    private final Drive drive;
    private final Shooter shooter;
    private final ApriltagVision vision;

    public ShootOnMove(Drive drive, Shooter shooter, ApriltagVision vision) {
        this.drive = drive;
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter, shooter, vision);
    }

@Override
public void execute() {
    // Get robot movement data
    double vx = drive.getChassisSpeeds().vxMetersPerSecond; // Sideways movement
    double vy = drive.getChassisSpeeds().vyMetersPerSecond; // Forward/backward movement

    // Get target distance from vision
    double d_target = vision.getBargeDistance(1); // In meters

    // Get shooter velocities (top and bottom)
    double v_shotT = shooter.getTopVelocity().in(RPM); // Shooter speed (m/s)
    double v_shotB = shooter.getBottomVelocity().in(RPM); // Shooter speed (m/s)
    double vExit = Math.sqrt(Math.pow(v_shotT, 2) + Math.pow(v_shotB, 2)); // Total exit velocity

    // Calculate the vertical component of the velocity
    double v_vertical = vExit * Math.sin(SHOOTER_ANGLE);

    // Calculate time of flight considering gravity (simplified)
    double t_flight = (2 * v_vertical) / G;

    // Motion compensation based on robot's velocity
    double x_comp = vx * t_flight; // Sideways correction
    double y_comp = vy * t_flight; // Forward/backward correction
    double d_corrected = d_target + y_comp;

    // Adjust shooter RPM based on corrected distance
    double v_needed = d_corrected / t_flight;
    double rpm_set = (v_needed / MAX_SHOT_SPEED) * MAX_RPM;

    // Calculate aiming offset for lateral motion
    double theta_adjust = Math.toDegrees(Math.atan(x_comp / d_corrected));

    var speeds = new ChassisSpeeds(vx, vy, theta_adjust);

    // Apply corrections
    shooter.shoot(RPM.of(rpm_set), RPM.of(rpm_set));
    drive.runVelocity(speeds);
}


    @Override
    public boolean isFinished() {
        return false; // Runs continuously during the match
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
