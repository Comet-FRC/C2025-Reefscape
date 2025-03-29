package frc.robot.commands.hoodtake;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromReefAuto extends WrapperCommand {
    public HoodtakeFromReefAuto(Drive drive, Hoodtake hoodtake, Supplier<TargetAlgae> targetAlgaeSupplier) {
        super(
            Commands.sequence(
                drive.pathfindToPose(targetAlgaeSupplier.get()::pose, 0)
                    .onlyWhile(() -> drive.getDistanceFrom(targetAlgaeSupplier.get().pose()).gt(Meters.of(0.5))),
                Commands.either(
                    new HoodtakeFromL3Auto(drive, hoodtake, targetAlgaeSupplier),
                    new HoodtakeFromL2Auto(drive, hoodtake, targetAlgaeSupplier),
                    () -> targetAlgaeSupplier.get().id() % 2 == 0
                ),
                Commands.waitSeconds(0.4),
                drive.pathfindToPose(targetAlgaeSupplier.get()::pose, 0)
                    .onlyWhile(() -> drive.getDistanceFrom(targetAlgaeSupplier.get().pose()).gt(Meters.of(0.2))),
                hoodtake.defaultCommand()
            )
        );
    }
}
