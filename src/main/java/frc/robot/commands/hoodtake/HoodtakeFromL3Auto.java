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

public class HoodtakeFromL3Auto extends SequentialCommandGroup {
    
    public HoodtakeFromL3Auto(Drive drive, Hoodtake hoodtake, Supplier<TargetAlgae> targetAlgaeSupplier) {
        super(
            // Commands.runOnce(() -> System.out.println("id: " + targetAlgaeSupplier.get().id())),
            // Commands.runOnce(() -> System.out.println("pose: " + targetAlgaeSupplier.get().pose())),
            // Commands.runOnce(() -> System.out.println("isRed: " + targetAlgaeSupplier.get().isRed())),
            hoodtake.setPivotPosition(() -> Degrees.of(100)),
            hoodtake.setWheelVoltage(() -> Volts.of(-5)),
            drive.driveToTargetAlgaePID(() -> Meters.of(0.59), targetAlgaeSupplier)
                .withTimeout(2),
            hoodtake.setPivotPosition(() -> Degrees.of(55)),
            Commands.waitUntil(hoodtake::atPosition)
                .withTimeout(1)
            
            // hoodtake.setPivotPosition(() -> Degrees.of(80))
            //     .andThen(Commands.waitUntil(hoodtake::atPosition))
            //drive.driveToClosestAlgaePID(() -> Meters.of(0.51))
        );
    }

    
}
