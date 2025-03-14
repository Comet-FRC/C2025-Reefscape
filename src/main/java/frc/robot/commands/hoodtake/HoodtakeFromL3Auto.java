package frc.robot.commands.hoodtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromL3Auto extends SequentialCommandGroup {
    
    public HoodtakeFromL3Auto(Drive drive, Hoodtake hoodtake) {
        super(
            Commands.deadline(
                drive.pathfindToPose(drive.getTargetAlgae()::pose, 1),
                Commands.sequence(
                    hoodtake.setWheelVoltage(() -> Volts.of(-3)),
                    hoodtake.setPivotPosition(() -> Degrees.of(55)),
                    Commands.waitUntil(hoodtake::atPosition)
                )
            ),
            drive.driveToClosestAlgaePID(() -> Meters.of(0.7))
            // hoodtake.setPivotPosition(() -> Degrees.of(80))
            //     .andThen(Commands.waitUntil(hoodtake::atPosition))
            //drive.driveToClosestAlgaePID(() -> Meters.of(0.51))
        );
    }

    
}
