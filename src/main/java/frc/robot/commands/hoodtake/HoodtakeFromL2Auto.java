package frc.robot.commands.hoodtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.subsystems.shooter.Shooter;

public class HoodtakeFromL2Auto extends SequentialCommandGroup {
    
    public HoodtakeFromL2Auto(Drive drive, Hoodtake hoodtake, Supplier<TargetAlgae> targetAlgaeSupplier) {
        super(
            // Commands.runOnce(() -> System.out.println("id: " + targetAlgae.get().id())),
            // Commands.runOnce(() -> System.out.println("pose: " + targetAlgae.get().pose())),
            // Commands.runOnce(() -> System.out.println("isRed: " + targetAlgae.get().isRed())),
            drive.pathfindToPose(targetAlgaeSupplier.get()::pose, 0)
            .onlyWhile(() -> drive.getDistanceFrom(targetAlgaeSupplier.get().pose()).gt(Meters.of(0.5))),
            drive.driveToTargetAlgaePID(() -> Meters.of(0.65), targetAlgaeSupplier)
                .withTimeout(4),
            hoodtake.setPivotPosition(() -> Degrees.of(45)),
            hoodtake.setWheelVoltage(() -> Volts.of(5)),
            Commands.waitUntil(hoodtake::atPosition)
                .withTimeout(1),
            Commands.waitSeconds(0.4),
            drive.pathfindToPose(targetAlgaeSupplier.get()::pose, 0)
                .onlyWhile(() -> drive.getDistanceFrom(targetAlgaeSupplier.get().pose()).gt(Meters.of(0.1)))
        );
    }

    
}
