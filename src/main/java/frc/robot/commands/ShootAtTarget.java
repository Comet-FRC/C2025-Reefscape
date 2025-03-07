package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.NetTargetSelector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.controller.CometController;

public class ShootAtTarget extends WrapperCommand {
    public ShootAtTarget(CometController controller, Drive drive, Shooter shooter) {
        super(
            Commands.sequence(
                Commands.deadline(
                    Commands.waitUntil(() -> shooter.isReadyToShoot()),
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
                    Commands.repeatingSequence(
                        shooter.setFlywheelVelocitiesFromDistance(() -> drive.getDistanceFrom(NetTargetSelector.getInstance().getTranslation()))
                    )
                ),
                Commands.runOnce(() -> controller.getHid().setRumble(GenericHID.RumbleType.kBothRumble, 0.5), shooter),
                Commands.waitUntil(() -> false)
            ).finallyDo(
                (interrupted) -> {
                    controller.getHid().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
                }
            )
        );
    }
}
