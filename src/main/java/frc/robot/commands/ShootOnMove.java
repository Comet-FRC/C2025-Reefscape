package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
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
    private final double boundX;
    private double distanceToNet;

    public ShootOnMove(Drive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;
        addRequirements(shooter);

        // All Barges have the same X value
        this.boundX = FieldConstants.Barge.startOfBlueBarge.getX();
        distanceToNet = 0;
    }

    @Override
    public void execute() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isEmpty()) return; // Exit early if no alliance data is available

        Translation2d bound1 = (ally.get() == Alliance.Blue) ? 
            FieldConstants.Barge.startOfBlueBarge : FieldConstants.Barge.startOfRedBarge;
        Translation2d bound2 = (ally.get() == Alliance.Blue) ? 
            FieldConstants.Barge.endOfBlueBarge : FieldConstants.Barge.endOfRedBarge;

        // Get robot movement data
        double vx = drive.getChassisSpeeds().vxMetersPerSecond; // Forward/backward movement
        double vy = drive.getChassisSpeeds().vyMetersPerSecond; // Sideways movement (Left is +)
        
    // Get distance to the net based on robot movement        
    if(vy > 0){
        distanceToNet = drive.getPose().getTranslation().getDistance(bound1);
        } else {
        distanceToNet = drive.getPose().getTranslation().getDistance(bound2);
    }
    
    boolean shootOnMove = distanceToNet/ vy < 1.5; // If the robot is within 1.5 seconds of the net, shoot
                                                   //TODO: Replace 1.5 with a more accurate value

    if(shootOnMove){
        double scalar = -1; 
        shooter.shootFromDistance(() -> {
            double omega = vx * scalar; //TODO: figure out scalar                                         

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
