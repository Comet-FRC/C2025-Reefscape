package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.subsystems.shooter.NetTargetSelector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.controller.CometController;

public class ShootAtTargetFromDistance extends WrapperCommand {
    public ShootAtTargetFromDistance(CometController controller, Drive drive, Shooter shooter, Hoodtake hoodtake) {
        super(
            Commands.parallel(
                Commands.either(
                    Commands.runOnce(() -> controller.getHid().setRumble(GenericHID.RumbleType.kBothRumble, 0.5)),
                    Commands.runOnce(() -> controller.getHid().setRumble(GenericHID.RumbleType.kBothRumble, 0)),
                    () -> shooter.isReadyToShoot()),
                drive.driveWithAngleSetpoint(
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> {
                        Translation2d netTranslation2d = NetTargetSelector.getInstance().getTranslation();
                        Translation2d robotTranslation2d = drive.getPose().getTranslation();
                        Translation2d diff = netTranslation2d.minus(robotTranslation2d);
                        double rotation = Math.atan2(diff.getY(),diff.getX());
                        return new Rotation2d(rotation);
                    }
                ),
                Commands.sequence(
				    hoodtake.setPivotPosition(() -> Degrees.of(90)),
				    shooter.setFlywheelVelocities(
                        () -> RPM.of(SmartDashboard.getNumber("Shooter/topSpeedRPM", 2000)),
                        () -> RPM.of(SmartDashboard.getNumber("Shooter/botSpeedRPM", 2000))
                    ),
				    // this.shooter.setBottomVoltage(() -> Volts.of(3)),
				    Commands.waitUntil(() -> false)
			    )
            )
            // .finallyDo(
            //     (interrupted) -> {
            //         controller.getHid().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            //     }
            // )
        );
    }
}
