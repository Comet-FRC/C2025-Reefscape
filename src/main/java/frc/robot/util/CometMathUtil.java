package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public final class CometMathUtil {
    private CometMathUtil() {
        throw new AssertionError("utility class");
    }

    /**
     * @see https://en.wikipedia.org/wiki/Feature_scaling
     * @param input
     * @param minInputValue
     * @param maxInputValue
     * @return
     */
    public static double minMaxScale(double input, double minInputValue, double maxInputValue) {
        if (Math.abs(input) < minInputValue)
            input = minInputValue;
        input = MathUtil.clamp(input, -maxInputValue, maxInputValue);
        return (input - minInputValue)/(maxInputValue - minInputValue);
    }
}
