package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import static edu.wpi.first.units.Units.*;
import java.util.Optional;

public class ShootOnMove extends Command {

    private final Drive drive;
    private final Shooter shooter;
    private double distanceToNet;
    Translation2d bound1, bound2;

    public ShootOnMove(Drive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;
        addRequirements(shooter);
        Optional<Alliance> ally = DriverStation.getAlliance();

        Translation2d bound1 = (ally.get() == Alliance.Blue) ? 
            FieldConstants.Barge.startOfBlueBarge : FieldConstants.Barge.startOfRedBarge;
        Translation2d bound2 = (ally.get() == Alliance.Blue) ? 
            FieldConstants.Barge.endOfBlueBarge : FieldConstants.Barge.endOfRedBarge;

        distanceToNet = 0;
    }

    @Override
    public void execute() {
        
  
        ChassisSpeeds deltaVelocity = drive.getFieldOrientedChassisSpeeds();
        // Get robot movement data
        double deltaFowardVelocity = deltaVelocity.vxMetersPerSecond; // Forward/backward movement
        double deltaSidewaysVelocity = deltaVelocity.vyMetersPerSecond; // Sideways movement (Left is +)
        
    // Get distance to the net based on robot Y movement        
    if(deltaSidewaysVelocity > 0.0){
        distanceToNet = drive.getPose().getTranslation().getDistance(bound1);
        } else {
        distanceToNet = drive.getPose().getTranslation().getDistance(bound2);
    }
    
    double t = 1.5; //TODO: change t
    boolean shootOnMove = distanceToNet/ deltaSidewaysVelocity < t; // If the robot is within t seconds of the net, shoot

    if(shootOnMove){
        double scalar = -1; 
        shooter.shootFromDistance(() -> {
            double omega = deltaFowardVelocity * scalar; //TODO: figure out scalar                                         

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
