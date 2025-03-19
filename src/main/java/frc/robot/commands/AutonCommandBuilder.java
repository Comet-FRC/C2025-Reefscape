package frc.robot.commands;

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
import frc.robot.util.AllianceColor;

public class AutonCommandBuilder {
    private static LoggedNetworkString strategyInput = new LoggedNetworkString("/SmartDashboard/Auton", "DEC");

    public static Command getCommand(Drive drive, Hoodtake hoodtake) {
        // System.out.println("Building Command");

        Command command = Commands.none();
        String input = strategyInput.get();

        for (int i=0; i<input.length(); ++i) {
            int algaeId = (input.toUpperCase().charAt(i) - 'A') % 6;
            Pose2d targetPose = FieldConstants.Reef.getTeamAlgaePoses()[algaeId];

            // System.out.println(targetPose);


            TargetAlgae targetAlgae = new TargetAlgae(targetPose, algaeId, AllianceColor.isRed());
            command = command.andThen(new HoodtakeFromReef(drive, hoodtake, () -> targetAlgae));
        }

        return command;
    }
}
