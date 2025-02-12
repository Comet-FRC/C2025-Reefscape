package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVisualizer3d {
    private final String name;
    private final Translation3d position;
    private final Rotation2d forwardDirection; 

    private Angle pivotAngle;

    /*private final Distance x;
    private final Distance y;
    private final Distance z;*/


    public ArmVisualizer3d(String name, Translation3d position, Rotation2d forwardDirection) {
        this.name = name;
        this.position = position;
        this.forwardDirection = forwardDirection;
       
    }

    public void setArmAngle(Angle angle) {
        pivotAngle = angle;
        Logger.recordOutput("Mechanism3D/" +     this.name + "Angle", angle);
    }

    public void publish() {
        Logger.recordOutput("Mechanism3D/" + this.name, new Pose3d(position, new Rotation3d(pivotAngle.in(Radians), 0, forwardDirection.getRadians())));
    }
}
