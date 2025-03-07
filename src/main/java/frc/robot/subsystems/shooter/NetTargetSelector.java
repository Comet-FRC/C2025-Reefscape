package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceColor;

public class NetTargetSelector {

    private static NetTargetSelector instance;

    public static NetTargetSelector getInstance() {
        if (instance == null) {
            instance = new NetTargetSelector();
        }
        return instance;
    }

    private double targetScalar;
    private Translation2d targetTranslation;

    private boolean isAlreadyCalculated;

    private final Translation2d leftBound;
    private final Translation2d rightBound;

    public NetTargetSelector() {
        this.targetScalar = 0.5;
        this.isAlreadyCalculated = false;

        if (AllianceColor.isRed()) {
            this.leftBound = FieldConstants.Barge.startOfRedBarge;
            this.rightBound = FieldConstants.Barge.endOfRedBarge;
        } else {
            this.leftBound = FieldConstants.Barge.startOfBlueBarge;
            this.rightBound = FieldConstants.Barge.endOfBlueBarge;
        }
    }

    public Translation2d getTranslation() {
        if (isAlreadyCalculated)
            return this.targetTranslation;

        Translation2d netDifference = this.rightBound.minus(this.leftBound);
        this.targetTranslation = this.leftBound.plus(netDifference.times(targetScalar));
        this.isAlreadyCalculated = true;

        return this.targetTranslation;
    }

    public void setTargetScalar(double targetScalar) {
        this.targetScalar = MathUtil.clamp(targetScalar, 0.0, 1.0);
        this.isAlreadyCalculated = false;
    }

    public void add(double percentage) {
        this.setTargetScalar(this.targetScalar + percentage);
    }
}
