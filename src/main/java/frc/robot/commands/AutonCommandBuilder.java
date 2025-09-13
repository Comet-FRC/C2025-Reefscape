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
import frc.robot.commands.hoodtake.HoodtakeFromReefAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.util.AllianceColor;

public class AutonCommandBuilder {
    private LoggedNetworkString strategyInput = new LoggedNetworkString("/SmartDashboard/Auton", "DEC");

    public Command getCommand(Drive drive, Hoodtake hoodtake) {
        // System.out.println("Building Command");

        Command command = Commands.none();
        String input = strategyInput.get();

        for (int i=0; i<input.length(); ++i) {
            char c = input.toUpperCase().charAt(i);

            if (Character.isDigit(c)) {
                int seconds = c - '0';
                command = command.andThen(Commands.waitSeconds(seconds));
                continue;
            }

            int algaeId = (c - 'A') % 6;
            Pose2d targetPose = FieldConstants.Reef.getTeamAlgaePoses()[algaeId];
            TargetAlgae targetAlgae = new TargetAlgae(targetPose, algaeId, AllianceColor.isRed());

            if (i == 0) {
                command = command.andThen(new HoodtakeFromReefAuto(drive, hoodtake, () -> targetAlgae));
                continue;
            }

            {
                command = command.andThen(
                    Commands.parallel(
                        new HoodtakeFromReefAuto(drive, hoodtake, () -> targetAlgae)
                            .andThen(hoodtake.setWheelVoltage(() -> Volts.of(1)))
                    )
                );
            }
        }

        return command;
    }
}
