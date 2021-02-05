package Autonomous.ImageProcessing;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Point;
import android.util.Log;

import java.util.ArrayList;

import Autonomous.HeadingVector;
import Autonomous.Rectangle;
import Autonomous.RingCount;
import Autonomous.Vector3;

/**
 * Created by robotics on 12/12/17.
 */

/*
    A class to process images and mainly find the columns of the cryptobox

 */
public class RingImageProcessor {
    public int imageWidth;
    public int imageHeight;
    private double percentRequiredInColumnToCheck;
    private int minimumColumnWidth;
    public static final int DESIRED_HEIGHT = 80;
    public static final int DESIRED_WIDTH = 80;
    public static final double CLOSE_UP_MIN_PERCENT_COLUMN_CHECK = 0.3;
    public static final double FAR_AWAY_MIN_PERCENT_COLUMN_CHECK = 0.1;
    public static final int RED_TEAM = 0, BLUE_TEAM = 1;
    public static final int LEFT = 0, CENTER = 1, RIGHT = 2;
    public static final int UNKNOWN = -500;
    public enum RING_COLOR {ORANGE};
    RING_COLOR colorToFind = RING_COLOR.ORANGE;
    private int team;

    private final int MIN_ORANGE_WIDTH = 4;
    private final int QUAD_STACK_MIN_HEIGHT = 10, SINGLE_STACK_MIN_HEIGHT = 4;

    /**
     * constructor for this class, you should not use this as you can not set the team's color to look for
     *
     * @param desiredHeight         this is an int of the image that will be processed. It is suggested that this is a rather
     *                              small number to reduce the required processing time
     * @param desiredWidth          like suggested above, keep this small to reduce processing time.
     */

    public RingImageProcessor(int desiredHeight, int desiredWidth){
        imageHeight = desiredHeight;
        imageWidth = desiredWidth;
        colorToFind = RING_COLOR.ORANGE;
    }

    /**
     * this function scales a Bitmap to the correct size
     * @param bmp the Bitmap to scale
     * @return a Bitmap at the right size
     */

    public Bitmap scaleBmp(Bitmap bmp){
        bmp = Bitmap.createScaledBitmap(bmp,imageWidth,imageHeight,true);
        Bitmap b = bmp.copy( Bitmap.Config.ARGB_8888 , true);
        return b;
    }

    public Rectangle getOrangeBox(Bitmap bmp, boolean showOrangePixelsOnScreen) {
        int width = bmp.getWidth(), height = bmp.getHeight()-60;
        int[] pixels = new int[height*width];
        bmp.getPixels(pixels, 0, width, 0, 30, width, height);
        Rectangle orangeBox = new Rectangle();
        Point topLeft = new Point(), topRight = new Point(), bottomLeft = new Point(), bottomRight = new Point();
        int orangePixelCount = 0;
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                int color = pixels[y * width + x];
                if(checkIfOrange(color)) {
                    if(orangePixelCount == 0) {
                        topLeft = new Point(x, y);
                        orangePixelCount++;
                    } else if(orangePixelCount == 1 && Math.abs(topLeft.y - y) <= 3 && Math.abs(topLeft.x - x) >= MIN_ORANGE_WIDTH) {
                        topRight = new Point(x, y);
                        orangePixelCount++;
                    } else if(orangePixelCount == 2 && Math.abs(topLeft.y - y) <= QUAD_STACK_MIN_HEIGHT+3 && Math.abs(topLeft.x - x) <= 3) {
                        bottomLeft = new Point(x, y);
                        orangePixelCount++;
                    } else if(orangePixelCount == 3 && Math.abs(bottomLeft.y - y) <= 3 && Math.abs(bottomLeft.x - x) >= MIN_ORANGE_WIDTH) {
                        bottomRight = new Point(x, y);
                        orangePixelCount++;
                    }
                }
            }
        }

        if(showOrangePixelsOnScreen) {
            bmp.getPixels(pixels, 0, width, 0, 30, width, height);
            showOrangePixels(pixels, bmp.getHeight(), bmp.getWidth(), Color.GREEN);
            bmp.setPixels(pixels, 0, width, 0, 30, width, height);
        }

        if(orangePixelCount == 4) {
            int x = (int) (((topRight.x - topLeft.x) / 2.0)+((bottomRight.x - bottomLeft.x) / 2.0) / 2.0 + 0.5);
            int y = (int) (((bottomLeft.y - topLeft.y) / 2.0)+((bottomRight.y - topRight.y) / 2.0) / 2.0 + 0.5);
            int newWidth = (int) ((Math.abs(topLeft.x - topRight.x) + Math.abs(bottomLeft.x - bottomRight.x)) / 2.0 + 0.5);
            int newHeight = (int) ((Math.abs(topLeft.y - bottomLeft.y) + Math.abs(topRight.y - bottomRight.y)) / 2.0 + 0.5);
            orangeBox = new Rectangle(x, y, newWidth, newHeight);
        }
        return orangeBox;
    }

    public int getNumOfRings(Bitmap bmp, boolean shouldModifyImage) {
        Rectangle orangeBoxOnScreen = getOrangeBox(bmp, shouldModifyImage);
        if(orangeBoxOnScreen.height != Double.MIN_VALUE) {
            if (orangeBoxOnScreen.height >= QUAD_STACK_MIN_HEIGHT) return 4;
            else if (orangeBoxOnScreen.height >= SINGLE_STACK_MIN_HEIGHT) return 1;
        }
        return -1;
    }

    public RingCount getRingCount(Bitmap bmp) {
        return getRingCount(bmp, false);
    }

    public RingCount getRingCount(Bitmap bmp, boolean shouldModifyImage) {
        RingCount r = RingCount.NO_RINGS;
        int numOfRings = getNumOfRings(bmp, shouldModifyImage);
        if(numOfRings == 1) r = RingCount.SINGLE_STACK;
        else if(numOfRings == 4) r = RingCount.QUAD_STACK;
        return r;
    }

    /*
        this function will show the orange pixels in an image
     */
    public void showOrangePixels(int [] pixels, int height, int width, int colorToReplaceWith) {
        for (int c = 0; c < width; c++) {
            int numberOfOrangePixels = 0;
            for (int r = 0; r < height; r++) {
                int color = pixels[r * width + c];
                int[] rgba = {Color.red(color), Color.blue(color), Color.green(color), Color.alpha(color)};
                float[] hsv = new float[3];
                Color.colorToHSV(color, hsv);
                //check for blue
                if (checkIfOrange(hsv)) {
                    rgba[0] = 0;
                    rgba[1] = 250;
                    rgba[2] = 250;
                    pixels[r * width + c] = colorToReplaceWith;
                    numberOfOrangePixels++;
                }
            }
        }
    }

    public boolean checkIfOrange(float [] hsl) {
        if (hsl[0] > 0 || hsl[0] < 360) {
            if (hsl[1] > .54) {
                if (hsl[2] > .33 && hsl[2] < .62) {
                    return true;
                }
            }
        }
        return false;
    }

    public boolean checkIfOrange(int color) {
        float [] hsl = new float[3];
        Color.colorToHSV(color, hsl);
        return checkIfOrange(hsl);
    }
}
