package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceColor;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class ShootOnMove extends Command {

    private final Drive drive;
    private final Shooter shooter;
    Translation2d minBound, maxBound;

    private final double SCALING_FACTOR = 0.5;
    private final double QUADRATIC_FACTOR = 0;

    public ShootOnMove(Drive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;

        if (AllianceColor.isRed()) {
            this.minBound = FieldConstants.Barge.startOfRedBarge;
            this.maxBound = FieldConstants.Barge.endOfRedBarge;
        } else {
            this.minBound = FieldConstants.Barge.startOfBlueBarge;
            this.maxBound = FieldConstants.Barge.endOfBlueBarge;
        }
    }

    @Override
    public void execute() {

        ChassisSpeeds chassisSpeeds = drive.getFieldOrientedChassisSpeeds();
        Angle robotDirection = drive.getPose().getRotation().getMeasure();
        
        LinearVelocity xVelocity = MetersPerSecond.of(chassisSpeeds.vxMetersPerSecond);
        LinearVelocity yVelocity = MetersPerSecond.of(chassisSpeeds.vyMetersPerSecond); // Sideways movement (Left is +)

        Distance xFromNet = this.minBound.getMeasureX().minus(drive.getPose().getMeasureX());
        Distance distanceFacingNet = xFromNet.div(Math.cos(drive.getPose().getRotation().getRadians()));
        
        /*
        TODO: if distanceFacingNet is negative, then the robot is facing away from the net
        We need to make sure that this isn't a problem...
        */

         // Get velocity in facing direction
        LinearVelocity forwardVelocity = xVelocity.times(Math.cos(robotDirection.in(Radians)))
            .plus(yVelocity.times(Math.sin(robotDirection.in(Radians))));

        // Apply an arbitrary scaling function to adjust distance
        Distance distanceAdjustment = arbitrateShooterDistance(forwardVelocity);
        Distance adjustedDistance = distanceFacingNet.minus(distanceAdjustment);

        Logger.recordOutput("ShootOnMove/distance adjustment", distanceAdjustment);
        Logger.recordOutput("ShootOnMove/adjusted distance", adjustedDistance);

        shooter.setFlywheelVelocitiesFromDistance(
            () -> adjustedDistance
        ).schedule();
    }

    private Distance arbitrateShooterDistance(LinearVelocity forwardVelocity) {
        double forwardSpeed = forwardVelocity.in(MetersPerSecond);

        double scalerAdjustment = forwardSpeed * SCALING_FACTOR;
        double quadraticAdjustment = Math.pow(forwardSpeed, 2) * QUADRATIC_FACTOR;

        Distance distanceAdjustment = Meters.of(scalerAdjustment + quadraticAdjustment);
        return distanceAdjustment;
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously during the match -> TODO: maybe add line checking indexer state
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
