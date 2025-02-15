package frc.robot.commands.hoodtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromL2 extends SequentialCommandGroup {
    
    public HoodtakeFromL2(Drive drive, Hoodtake hoodtake) {
        super(
            Commands.deadline(
                drive.pathfindToPose(drive.getTargetAlgae()::pose),
                hoodtake.setPosition(() -> Degrees.of(50))
            ),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.50)),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.6))
        );
    }

    
}
