package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVisualizer {
    private final String name;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d arm;

    public ArmVisualizer(String name, Distance armLength) {
        this.name = name;
        mechanism = new LoggedMechanism2d(60, 60);
        root = mechanism.getRoot(this.name + "Root", 30, 30);
        arm = root.append(new LoggedMechanismLigament2d("Arm", armLength.in(Meters), 90, 6, new Color8Bit(Color.kBlue)));
    }

    public void setArmAngle(Angle angle) {
        arm.setAngle(angle.in(Degrees));
        Logger.recordOutput("Mechanism2D/" + this.name + "Angle", angle);
    }

    public void setArmLength(Distance length) {
        arm.setLength(length.in(Meters));
        Logger.recordOutput("Mechanism2D/" + this.name + "Length", length.in(Meters));
    }

    public void publish() {
        Logger.recordOutput("Mechanism2D" + this.name, mechanism);
    }
}
