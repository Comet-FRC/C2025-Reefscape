package frc.robot.commands.hoodtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromL3Auto extends SequentialCommandGroup {
    
    public HoodtakeFromL3Auto(Drive drive, Hoodtake hoodtake, Supplier<TargetAlgae> targetAlgae) {
        super(
            drive.pathfindToPose(targetAlgae.get()::pose, 0),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.6)),
            hoodtake.setPivotPosition(() -> Degrees.of(55)),
            hoodtake.setWheelVoltage(() -> Volts.of(-5))
            // hoodtake.setPivotPosition(() -> Degrees.of(80))
            //     .andThen(Commands.waitUntil(hoodtake::atPosition))
            //drive.driveToClosestAlgaePID(() -> Meters.of(0.51))
        );
    }

    
}
