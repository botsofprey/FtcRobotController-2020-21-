package Autonomous.ImageProcessing;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Point;
import android.util.Log;

import java.util.ArrayList;
import java.util.Arrays;

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
    public static final int PIXEL_THRESHHOLD = 5;
    public enum RING_COLOR {ORANGE};
    RING_COLOR colorToFind = RING_COLOR.ORANGE;
    private int team;

    private final int MIN_ORANGE_WIDTH = 5;
    private final int QUAD_STACK_MIN_HEIGHT = 5, SINGLE_STACK_MIN_HEIGHT = 1;

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

    public int getNumOfRings(Bitmap bmp, boolean showOrangePixelsOnScreen) {
        int width = bmp.getWidth(), height = bmp.getHeight();
        int[] pixels = new int[height*width];
        int[] pixelsx = new int[width];
        int[] pixelsy = new int[height];
        bmp.getPixels(pixels, 0, width, 0, 0, width, height);
        for(int y = 31; y < height-38; y++) {
            for(int x = 19; x < width-43; x++) {
                int color = pixels[y * width + x];
                if(checkIfOrange(color)) {
                    pixelsx[x]++;
                    pixelsy[y]++;
                }
            }
        }
        int xinit = -1;
        int xfinal = -1;
        int yinit = -1;
        int yfinal = -1;
        for(int x = 0; x < width; x++) {
            if(pixelsx[x] > PIXEL_THRESHHOLD) {
                xinit = x;
                break;
            }
        }
        for(int x = width; x > 0; x--){
            if(pixelsx[x - 1] > PIXEL_THRESHHOLD) {
                xfinal = x-1;
                break;
            }
        }
        for(int y = 0; y < height; y++) {
            if(pixelsy[y] > PIXEL_THRESHHOLD) {
                yinit = y;
                break;
            }
        }
        for(int y = height; y > 0; y--){
            if(pixelsy[y - 1] > PIXEL_THRESHHOLD) {
                yfinal = y-1;
                break;
            }
        }

        if(showOrangePixelsOnScreen) {
            bmp.getPixels(pixels, 0, width, 0, 0, width, height);
            showOrangePixels(pixels, height, width, Color.GREEN);
            bmp.setPixels(pixels, 0, width, 0, 0, width, height);
        }

        Log.d("Rectangle x final", xfinal+"");
        Log.d("Rectangle x init", xinit+"");
        Log.d("Rectangle y final", yfinal+"");
        Log.d("Rectangle y init", yinit+"");
        Log.d("RectangleHistX", Arrays.toString(pixelsx));
        Log.d("RectangleHistY", Arrays.toString(pixelsy));
        if(yinit != -1) {
            if(xinit != -1) return 4;
            else return 1;
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
        if (hsl[0] > 7 || hsl[0] < 360) {
            if (hsl[1] > .57) {
                if (hsl[2] > .5 && hsl[2] < .72) {
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
