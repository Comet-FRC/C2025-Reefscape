package frc.robot.commands.hoodtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromL3 extends SequentialCommandGroup {
    
    public HoodtakeFromL3(Drive drive, Hoodtake hoodtake) {
        super(
            Commands.deadline(
                drive.pathfindToPose(drive.getTargetAlgae()::pose, 1),
                hoodtake.setPivotPosition(() -> Degrees.of(63))
                    .andThen(Commands.waitUntil(hoodtake::atPosition))
            ),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.44)),
            hoodtake.setPivotPosition(() -> Degrees.of(80))
                .andThen(Commands.waitUntil(hoodtake::atPosition)),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.51))
        );
    }

    
}
