package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.hoodtake.HoodtakeFromReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceColor;

public class AutonCommandBuilder {
    private LoggedNetworkString strategyInput = new LoggedNetworkString("/SmartDashboard/Auton", "D");

    public Command getCommand(Drive drive, Hoodtake hoodtake, Shooter shooter) {
        // System.out.println("Building Command");

        Command command = Commands.none();
        String input = strategyInput.get();

        for (int i=0; i<input.length(); ++i) {


            // System.out.println(input.toUpperCase().charAt(i));

            int algaeId = (input.toUpperCase().charAt(i) - 'A') % 6;
            Pose2d targetPose = FieldConstants.Reef.getTeamAlgaePoses()[algaeId];

            TargetAlgae targetAlgae = new TargetAlgae(targetPose, algaeId, AllianceColor.isRed());

            // System.out.println("targetPose: " + targetPose);
            // System.out.println("algaeId: " + algaeId);
            // System.out.println("isRed: " + AllianceColor.isRed());

            if (i == 0) {
                command = command.andThen(new HoodtakeFromReef(drive, hoodtake, () -> targetAlgae));
            } else {
                command = command.andThen(
                    Commands.parallel(
                        new HoodtakeFromReef(drive, hoodtake, () -> targetAlgae)
                        .andThen(hoodtake.setWheelVoltage(() -> Volts.of(1))),
                        Commands.sequence(
                            shooter.setBottomVoltage(() -> Volts.of(1)),
                            Commands.waitSeconds(1.0),
                            shooter.setBottomVoltage(() -> Volts.of(0))
                        )
                    )
                );
            }
            
        }

        return command;
    }
}
