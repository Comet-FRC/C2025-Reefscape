package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Field;
import java.util.Optional;

import com.fasterxml.jackson.databind.util.BeanUtil;


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
        addRequirements(shooter, vision);
    }

@Override
public void execute() {
    // Get robot movement data
    double vx = drive.getChassisSpeeds().vxMetersPerSecond; // Fowards/backward movement
    double vy = drive.getChassisSpeeds().vyMetersPerSecond; // Sideways movement (Left is +)

    // Get target distance from vision
    Translation2d target;
    Optional<Alliance> ally = DriverStation.getAlliance();
    if( ally.get() == Alliance.Blue) {
             target = FieldConstants.Barge.middleCage;
    } else {
            target = FieldConstants.Barge.midOfRedBarge;
    }

    if((Drive.getInstance().getPose().getX() > FieldConstants.Barge.middleCage.getX() && vx < 0) || 
    (Drive.getInstance().getPose().getX() < FieldConstants.Barge.middleCage.getX() && vx > 0))
    {
        double scalar = 0;
        
        shooter.shootFromDistance(() -> {
            double omega = vx * scalar;
            return Meters.of(drive.getPose().getX() + omega);
        });
    }

    
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
