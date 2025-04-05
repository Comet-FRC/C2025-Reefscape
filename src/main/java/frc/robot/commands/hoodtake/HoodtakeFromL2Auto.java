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
            Commands.deadline(
                drive.driveToTargetAlgaePID(() -> Meters.of(0.64), targetAlgaeSupplier)
                    .withTimeout(2),
                hoodtake.setPivotPosition(() -> Degrees.of(100))
                    .andThen(hoodtake.setWheelVoltage(() -> Volts.of(5)))
            ),
            hoodtake.setPivotPosition(() -> Degrees.of(44)),
            Commands.waitUntil(hoodtake::atPosition)
                .withTimeout(1)
        );
    }

    
}
