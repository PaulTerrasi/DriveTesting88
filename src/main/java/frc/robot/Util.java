package frc.robot;

/**
 * Utility functions that do not belong to any one specific class
 */
public class Util {

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
}