package frc.robot.commands;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
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

    private final Time maxTime = Seconds.of(20000000); // TODO: change t

    public ShootOnMove(Drive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;
        addRequirements(shooter);

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
        LinearVelocity yVelocity = MetersPerSecond.of(chassisSpeeds.vyMetersPerSecond); // Sideways movement (Left is +)

        Distance xFromNet = this.minBound.getMeasureX().minus(drive.getPose().getMeasureX());
        Distance distanceFacingNet = xFromNet.div(Math.cos(drive.getPose().getRotation().getRadians()));
        
        /*
        TODO: if distanceFacingNet is negative, then the robot is facing away from the net
        We need to make sure that this isn't a problem...
        */
        
        Distance yDeltaRobotProjection = distanceFacingNet.times(Math.sin(drive.getPose().getRotation().getRadians()));
        Distance yRobotProjection = drive.getPose().getMeasureY().plus(yDeltaRobotProjection);

        Distance yFromMaxBound = maxBound.getMeasureY().minus(yRobotProjection);

        Logger.recordOutput("ShootOnMove/1yFromNet", xFromNet);
        Logger.recordOutput("ShootOnMove/2distanceFacingNet", distanceFacingNet);
        Logger.recordOutput("ShootOnMove/3yDeltaRobotProjection", yDeltaRobotProjection);
        Logger.recordOutput("ShootOnMove/4yRobotProjection", yRobotProjection);
        Logger.recordOutput("ShootOnMove/5yFromMaxBound", yFromMaxBound);

        Time projectileYAirTime = yFromMaxBound.div(yVelocity); // Time for projectile to travel parallel to the net edge
        boolean acceptable = projectileYAirTime.lt(maxTime); // If the robot is within t seconds of the net, shoot

        shooter.shootFromDistance(
            () -> arbitrateShooterDistance(chassisSpeeds, distanceFacingNet, robotDirection)
        );
        Logger.recordOutput("ShootOnMove/6 Y Time", projectileYAirTime);
        Logger.recordOutput("ShootOnMove/7 acceptable Y Time", acceptable);
    }

    private Distance arbitrateShooterDistance(ChassisSpeeds speeds, Distance distanceFacingNet, Angle robotDirection) {
        double scalar = -2;
        double arbitratedAddendX, arbitratedAddendY;
        arbitratedAddendX = distanceFacingNet.times(Math.cos(robotDirection.in(Radians))).in(Meters);
        arbitratedAddendX = arbitratedAddendX + speeds.vxMetersPerSecond * scalar;
        arbitratedAddendY = distanceFacingNet.times(Math.sin(robotDirection.in(Radians))).in(Meters);
        arbitratedAddendY = arbitratedAddendY + speeds.vyMetersPerSecond * scalar;
        double arbitratedAddend = Math.hypot(arbitratedAddendX, arbitratedAddendY);
        return Meters.of(arbitratedAddend); // TODO: figure out scalar
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
