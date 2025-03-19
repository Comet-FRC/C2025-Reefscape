package frc.robot.commands.hoodtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromL2Auto extends SequentialCommandGroup {
    
    public HoodtakeFromL2Auto(Drive drive, Hoodtake hoodtake, Supplier<TargetAlgae> targetAlgae) {
        super(
            drive.pathfindToPose(targetAlgae.get()::pose, 0),
            drive.driveToTargetAlgaePID(() -> Meters.of(0.65), targetAlgae),
            hoodtake.setPivotPosition(() -> Degrees.of(45)),
            hoodtake.setWheelVoltage(() -> Volts.of(5))
            // drive.driveToClosestAlgaePID(() -> Meters.of(0.6))
        );
    }

    
}
