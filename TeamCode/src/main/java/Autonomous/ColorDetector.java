package Autonomous;

import android.graphics.Bitmap;

/**
 * Author: Ethan Fisher
 * Date: 10/26/2020
 *
 * Detects rings easily
 */

public class ColorDetector {

    public static final int TARGET_WIDTH = 125;
    public static final int TARGET_HEIGHT = 125;

    public static final int TRIES_TO_GET_NUM_RINGS = 25;

    public static final int ONE_RING_THRESHOLD = 45;
    public static final int FOUR_RING_THRESHOLD = 140;

    private VuforiaHelper vuforia;
    public int targetR;
    public int targetG;
    public int targetB;
    public int tolerance;

    public static ColorDetector ringDetector(VuforiaHelper vuforia) {
        return new ColorDetector(vuforia, 172, 93, 0, 50);
    }

    public ColorDetector(VuforiaHelper vuforia, int targetR, int targetG, int targetB, int tolerance) {
        this.vuforia = vuforia;
        this.targetR = targetR;
        this.targetG = targetG;
        this.targetB = targetB;
        this.tolerance = tolerance;
    }

    public int getNumRingsFound() {
        return getNumRingsFound(TRIES_TO_GET_NUM_RINGS);
    }

    public int getNumRingsFound(int numTries) {

        // number of rings
        int numZero = 0;
        int numOne = 0;
        int numFour = 0;

        // read 10 times or something like that
        for (int i = 0; i < numTries; i++) {
            int orangePixels = findNumDesiredPixels();

            if (orangePixels <= ONE_RING_THRESHOLD)
                numZero++;
            else if (orangePixels < FOUR_RING_THRESHOLD)
                numOne++;
            else
                numFour++;

        }

        if (numZero >= numOne && numZero >= numFour)
            return 0;
        if (numOne >= numZero && numOne >= numFour)
            return 1;
        return 4;
    }

    public int findNumDesiredPixels() {

        Bitmap image = vuforia.getImage(TARGET_WIDTH, TARGET_HEIGHT);
        if (image == null) return 0;

        // RGB for orange = #FFa500
        // variables for the bounds of the orange allowed for each pixel
        int redMin = targetR - tolerance;
        int redMax = targetR + tolerance;
        int greenMin = targetG - tolerance;
        int greenMax = targetG + tolerance;
        int blueMin = targetB - tolerance;
        int blueMax = targetB + tolerance;

        int orangePixels = 0;

        for (int i = 0; i < image.getWidth(); i++)
            for (int j = 0; j < image.getHeight(); j++) {
                int pixel = image.getPixel(i, j);

                // convert pixel to a color
//                int A = (pixel >> 24) & 0xff; // or color >>> 24
                int R = (pixel >> 16) & 0xff;
                int G = (pixel >>  8) & 0xff;
                int B = (pixel      ) & 0xff;

                if (R >= redMin && R <= redMax && G >= greenMin && G <= greenMax && B >= blueMin && B <= blueMax)
                    orangePixels++;
            }
        return orangePixels;
    }

}
