package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class IntakeFromL3 extends SequentialCommandGroup {
    
    public IntakeFromL3(Drive drive, Hoodtake hoodtake) {
        super(
            Commands.deadline(
                drive.pathfindToPose(drive.getTargetAlgae()::pose),
                hoodtake.setPosition(() -> Degrees.of(63))
            ),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.44)),
            hoodtake.setPosition(() -> Degrees.of(80)),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.51))
        );
    }

    
}
