package frc.robot;

/**
 * Utility functions that do not belong to any one specific class
 */
public class Util {

    public static final double EPSILON = 0.001;

    /**
     * Returns the base to the power of the exp, except even powers preserve sign
     */
    public static double signedPow(double base, int exp) {
        double unsignedPow = Math.pow(base, exp);
        if (exp % 2 == 0 && base < 0) {
            // Should be inverted
            return -unsignedPow;
        } else {
            return unsignedPow;
        }
    }

    /**
     * Returns the base, scaled by the given exponent while thresholding its 
     * lowest possible absolute value.
     */
    public static double signedThresholdedPow(double base, int exp, 
            double min, double max) {
        if (Math.abs(base) < EPSILON) {
            double ret = base * (max - min);
            Util.signedPow(ret, exp);
            ret += .075 * Math.signum(ret);

            return ret;
        } else {
            return 0;
        }
    }
}