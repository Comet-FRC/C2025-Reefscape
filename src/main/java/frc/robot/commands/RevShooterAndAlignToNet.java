package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceColor;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.controller.CometController;

public class RevShooterAndAlignToNet extends SequentialCommandGroup {

    private static final Distance BARGE_X = FieldConstants.Barge.BARGE_X;
    private static final LoggedTunableNumber SHOOTER_DISTANCE = new LoggedTunableNumber("Shooter/Net Distance", 1.9);

    public RevShooterAndAlignToNet(CometController controller, Hoodtake hoodtake, Shooter shooter, Drive drive) {
        super(
            hoodtake.setPivotPosition(() -> Degrees.of(90)),
            shooter.setFlywheelVelocities(
                () -> RPM.of(SmartDashboard.getNumber("Shooter/topSpeedRPM", 2000)),
                () -> RPM.of(SmartDashboard.getNumber("Shooter/botSpeedRPM", 2000))
            ),
            drive.driveWithXandAngleSetpoint(
                () -> {
                    Distance distanceFromNet = Meters.of(SHOOTER_DISTANCE.get());
                    // System.out.println(SHOOTER_DISTANCE.get());

                    if (drive.getPose().getMeasureX().gt(BARGE_X)) {
                        return BARGE_X.plus(distanceFromNet).in(Meters);
                    } else {
                        return BARGE_X.minus(distanceFromNet).in(Meters);
                    }
                },
                () -> -controller.getLeftX(),
                () -> {
                    if (drive.isOnOpposingSide()) 
                        return Rotation2d.fromDegrees(180);
                    else
                        return Rotation2d.kZero;
                })
        );
    }
    
}
