package Autonomous.ImageProcessing;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;

import java.util.ArrayList;

/**
 * Created by robotics on 12/12/17.
 */

/*
    A class to process images and mainly find the columns of the cryptobox

 */
public class SkystoneImageProcessor {
    public int imageWidth;
    public int imageHeight;
    private double percentRequiredInColumnToCheck;
    private int minimumColumnWidth;
    public static final int DESIRED_HEIGHT = 40;
    public static final int DESIRED_WIDTH = 40;
    public static final double CLOSE_UP_MIN_PERCENT_COLUMN_CHECK = 0.3;
    public static final double FAR_AWAY_MIN_PERCENT_COLUMN_CHECK = 0.1;
    public static final int RED_TEAM = 0, BLUE_TEAM = 1;
    public static final int LEFT = 0, CENTER = 1, RIGHT = 2;
    public static final int UNKNOWN = -500;
    public enum STONE_COLOR {YELLOW,BLACK};
    STONE_COLOR colorToFind = STONE_COLOR.BLACK;
    private int team;

    /**
     * constructor for this class, you should not use this as you can not set the team's color to look for
     *
     * @param desiredHeight         this is an int of the image that will be processed. It is suggested that this is a rather
     *                              small number to reduce the required processing time
     * @param desiredWidth          like suggested above, keep this small to reduce processing time.
     * @param percentColumnCheck    this is the minimum required percentage of one pixel column that must be the team's color to consider
     * @param minColumnWidth        this is the minimum required number of columns to be of the required color next to each other before the algorithm concludes the columns to be a cryptobox column
     */

    public SkystoneImageProcessor(int desiredHeight, int desiredWidth, double percentColumnCheck, int minColumnWidth){
        imageHeight = desiredHeight;
        imageWidth = desiredWidth;
        percentRequiredInColumnToCheck = percentColumnCheck;
        minimumColumnWidth = minColumnWidth;
    }

    /**
     *
     * @param desiredHeight         this is an int of the image that will be processed. It is suggested that this is a rather
     *                              small number to reduce the required processing time
     * @param desiredWidth          like suggested above, keep this small to reduce processing time.
     * @param percentColumnCheck    this is the minimum required percentage of one pixel column that must be the team's color to consider
     * @param minColumnWidth        this is the minimum required number of columns to be of the required color next to each other before the algorithm concludes the columns to be a cryptobox column
     * @param color                 This is the team's color, which is also the color cryptobox the algorithm will be looking for
     */
    public SkystoneImageProcessor(int desiredHeight, int desiredWidth, double percentColumnCheck, int minColumnWidth, STONE_COLOR color){
        colorToFind = color;
        imageHeight = desiredHeight;
        imageWidth = desiredWidth;
        percentRequiredInColumnToCheck = percentColumnCheck;
        minimumColumnWidth = minColumnWidth;
    }

    /**
     *
     * @param desiredHeight         this is an int of the image that will be processed. It is suggested that this is a rather
     *                              small number to reduce the required processing time
     * @param desiredWidth          like suggested above, keep this small to reduce processing time.
     * @param percentColumnCheck    this is the minimum required percentage of one pixel column that must be the team's color to consider
     * @param minColumnWidth        this is the minimum required number of columns to be of the required color next to each other before the algorithm concludes the columns to be a cryptobox column
     * @param color                 This is the team's color, which is also the color cryptobox the algorithm will be looking for
     * @param team                  This represents the alliance you are part of
     */
    public SkystoneImageProcessor(int desiredHeight, int desiredWidth, double percentColumnCheck, int minColumnWidth, STONE_COLOR color, int team){
        colorToFind = color;
        imageHeight = desiredHeight;
        imageWidth = desiredWidth;
        percentRequiredInColumnToCheck = percentColumnCheck;
        minimumColumnWidth = minColumnWidth;
        this.team = team;
    }
    /**
     * this function sets the required minimum columns that need to be adjacent and of the same color before the algorithm concludes
     * the adjacent columns are part of a cryptobox column
     * @param percentOfWidthOfImage the minimum number of adjacent columns of the team's color before being called a cryptobox column
     */

    public void setRequiredMinimumColumnWidth(double percentOfWidthOfImage){
        int widthInPixels = (int) (percentOfWidthOfImage * imageWidth + .5);
        if (widthInPixels <= 0) widthInPixels = 1;
        else if (widthInPixels > 1) throw new RuntimeException("Wanted Column Width Greater Than Image Width!");
        minimumColumnWidth = widthInPixels;
    }

    /**
     * this function sets the minimum percentage of a column of the camera's image to be the team's color before deciding the column is of
     * that color.
     * @param percentOfImageHeight  the minimum percentage of a column to be the team's color before calling the column a cryptobox column
     */

    public void setRequiredPercentOfImageColumnBlue(double percentOfImageHeight) {
        if (percentOfImageHeight > 1)
            throw new RuntimeException("Wanted Column Height Percent Greater Than 1.0!");
        percentRequiredInColumnToCheck = percentOfImageHeight;
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

    private ArrayList<Integer> getColumnsWithRequiredPercentBlack(double [] blackFrequencyByColumn){
        ArrayList<Integer> interestingColumns = new ArrayList<Integer>();
        //look for columns that have the minimum required % of a color
        for(int i = 0; i < blackFrequencyByColumn.length; i++){
            if(blackFrequencyByColumn[i] >= percentRequiredInColumnToCheck){
                interestingColumns.add(Integer.valueOf(i));
            }
        }
        return interestingColumns;
    }

    private ArrayList<Integer> getColumnsWithRequiredBlackCount(int[] blackFrequencyByColumn){
        ArrayList<Integer> interestingColumns = new ArrayList<Integer>();
        //look for columns that have the minimum required % of a color
        int minimumPixelCount = (int)(percentRequiredInColumnToCheck * imageHeight + .5);
        for(int i = 0; i < blackFrequencyByColumn.length; i++){
            if(blackFrequencyByColumn[i] >= minimumPixelCount){
                interestingColumns.add(Integer.valueOf(i));
            }
        }
        return interestingColumns;
    }

    /**
     * this function determines the bounds of the skystone in a image
     * @param columnsWithRequiredBlackPercent the indexes of columns determined to meet the minimum percentage of the team's color
     * @return an ArrayList of the start and end of the columns of the image that correspond to the start and end of the skystone
     */

    private ArrayList<Integer> getColumnBounds(ArrayList<Integer> columnsWithRequiredBlackPercent){
        ArrayList<Integer> columnBounds = new ArrayList<Integer>();
        if(columnsWithRequiredBlackPercent.size() > 0) {
            columnBounds.add(columnsWithRequiredBlackPercent.get(0));
            for (int i = 1; i < columnsWithRequiredBlackPercent.size(); i++) {
                if (i != 1) columnBounds.add(columnsWithRequiredBlackPercent.get(i));
                while (i < columnsWithRequiredBlackPercent.size() && columnsWithRequiredBlackPercent.get(i).intValue() == columnsWithRequiredBlackPercent.get(i - 1).intValue() + 1) {
                    i++;
                }
                columnBounds.add(columnsWithRequiredBlackPercent.get(i - 1));
            }
            //look for columns to be at least ___ columns wide
            for (int i = 0; i < columnBounds.size() - 1; i += 2) {
                if (columnBounds.get(i + 1).intValue() - columnBounds.get(i).intValue() >= minimumColumnWidth) {
                } else {
                    columnBounds.remove(i);
                    columnBounds.remove(i);
                    i -= 2;
                }
            }
        }
        return columnBounds;
    }

    private ArrayList<Integer> getYellowColumnBounds(ArrayList<Integer> columnsWithRequiredYellowPercent) {
        ArrayList<Integer> columnBounds = new ArrayList<>();
        if(columnsWithRequiredYellowPercent.size() > 1) {
            columnBounds.add(columnsWithRequiredYellowPercent.get(0));
            columnBounds.add(columnsWithRequiredYellowPercent.get(columnsWithRequiredYellowPercent.size()/2-1));
            columnBounds.add(columnsWithRequiredYellowPercent.get(columnsWithRequiredYellowPercent.size()/2));
            columnBounds.add(columnsWithRequiredYellowPercent.get(columnsWithRequiredYellowPercent.size()-1));
        }

        return columnBounds;
    }

    /**
     * this function determines the centers of the colored cryptobox columns. This can only be used if two columns are in the view of the camera.
     * as this is typically not the case, it is suggested that you use the findColumnCenters function
     * @param columnBounds the processed and determined bounds of cryptoboxes.
     * @return an ArrayList of the centers of the cryptobox, where you would score a glyph
     */

    private ArrayList<Integer> getColumnCenters(ArrayList<Integer> columnBounds) {
        ArrayList<Integer> columnCenters = new ArrayList<Integer>();
        for (int i = 0; i < columnBounds.size() / 2; i++) {
            columnCenters.add((int) (columnBounds.get(i * 2).intValue() + (columnBounds.get(i * 2 + 1).intValue() - columnBounds.get(i * 2).intValue()) / 2.0 + .5));
        }
        return columnCenters;
    }

    /**
     * This function returns the location of the colored cryptobox columns
     * @param bmp the Bitmap from the camera
     * @param shouldModifyImage whether the algorithm should modify the image as it processes it. Set this to true if you intend on saving the image
     * @return an ArrayList of the columns, relative to the width of the Bitmap, of the location of all seen cryptobox columns
     */

    public ArrayList<Integer> findColumnCenters(Bitmap bmp, boolean shouldModifyImage){
        ArrayList<Integer> columns = findColumns(bmp,shouldModifyImage);
        ArrayList<Integer> centers = new ArrayList<Integer>();
        for(int i = 0; i < columns.size() -1; i ++){
            centers.add(Integer.valueOf((int)((columns.get(i) + columns.get(i + 1))/2.0 + .5)));
        }
        return centers;
    }

    /**
     * This is the basis of the entire cryptobox processing image algorithm. This algorithm works by first determining all
     * columns that are of the team's color. This is determined through comparing the percentage of the coloumn that are blue or red to the minimum required percentage.
     * Then those columns that are blue or red are compared to columns adjacent to them to determine if there is a colored column or if it is noise.
     * @param bmp
     * @param shouldModifyImage
     * @return
     */

    public ArrayList<Integer> findColumns(Bitmap bmp, boolean shouldModifyImage) {
        //Log.d("CF IMG ","Height: " + bmp.getHeight() + " Width: " + bmp.getWidth());
        if (bmp.getHeight() > imageHeight && bmp.getWidth() > imageWidth){
            bmp = scaleBmp(bmp);
        }

        ArrayList<Integer> columnCenters;

        int width = bmp.getWidth(), height = bmp.getHeight();
        int newHeight = (int)((1.0/4.0)*height);
        int[] pixels = new int[width * height];
        int x = 0, y = (int)((3.0/4.0)*height), w = width, h = newHeight;

        bmp.getPixels(pixels, 0, width, x, y, w, h);

        int [] frequencyByColumn = null;
        if (colorToFind == STONE_COLOR.BLACK)
            frequencyByColumn = collapseVerticallyByBlackCount(pixels,w,h);
        else if (colorToFind == STONE_COLOR.YELLOW)
            frequencyByColumn = collapseVerticallyByYellowCount(pixels,w,h);

        ArrayList<Integer> interestingColumns = getColumnsWithRequiredBlackCount(frequencyByColumn);

        ArrayList<Integer> columnBounds = (colorToFind == STONE_COLOR.YELLOW) ? getYellowColumnBounds(interestingColumns) : getColumnBounds(interestingColumns);
//        for (int i = 0; i < columnBounds.size(); i++) {
//            Log.d("Bound", columnBounds.get(i).toString());
//        }
        //get average column locations
        columnCenters = getColumnCenters(columnBounds);
//        for (int i = 0; i < columnCenters.size(); i++) {
//            Log.d("Centers", columnCenters.get(i).toString());
//        }
//        Log.d("# of Columns", "" + columnCenters.size());
        if(shouldModifyImage){
            if(colorToFind == STONE_COLOR.BLACK) showBlackPixels(pixels,newHeight,width, Color.GREEN);
            else if(colorToFind == STONE_COLOR.YELLOW) showYellowPixels(pixels,newHeight,width, Color.GREEN);
            showColumnCenters(pixels,newHeight,width,columnCenters, Color.RED);
            bmp.setPixels(pixels, 0, width, 0, 0, width, height);
        }

        if (columnCenters.size() > 0) return columnCenters;
        return null;
    }

    /**
     * This function modifies the pixels to show where the center of the colored columns are
     */

    private void showColumnCenters(int [] pixels, int height, int width, ArrayList<Integer> columnCenters, int colorToShowWith) {
        for (int i = 0; i < columnCenters.size(); i++) {
            for (int r = 0; r < height; r++) {
                pixels[r * width + columnCenters.get(i)] = colorToShowWith;
            }
        }
    }

    /*
    *  This function finds if it is left, center, or right Skystone
    */

    public int findSkystonePixelLocation(Bitmap bmp) {
        ArrayList<Integer> columnCenters = findColumnCenters(bmp, false);
        if(columnCenters == null) return -1;
        else {
            int center = columnCenters.get(0);
            return Math.abs(center - (int) (bmp.getWidth() / 2.0));
        }
    }

    public int getSkystoneRelativePosition(Bitmap bmp) {
        colorToFind = STONE_COLOR.BLACK;
        ArrayList<Integer> skystoneCenters = findColumns(bmp, false);
        int skystoneCenter = UNKNOWN;
        if(skystoneCenters != null && skystoneCenters.size() > 0) skystoneCenter = skystoneCenters.get(0);

        colorToFind = STONE_COLOR.YELLOW;
        ArrayList<Integer> stoneCenters = findColumns(bmp, false);

        if(skystoneCenter != UNKNOWN) {
            if(stoneCenters != null && stoneCenters.size() > 1) {
                if (skystoneCenter > stoneCenters.get(1)) {
                    return RIGHT;
                } else if (skystoneCenter > stoneCenters.get(0)) {
                    return CENTER;
                } else {
                    return LEFT;
                }
            }
            return UNKNOWN;
        } else if(stoneCenters != null && stoneCenters.size() > 1) {
            colorToFind = STONE_COLOR.YELLOW;
            int width = bmp.getWidth(), height = bmp.getHeight();
            int[] pixels = new int[width * height];
            int newHeight = (int)((1.0/4.0)*height);
            bmp.getPixels(pixels, 0, width, 0, (int)((3.0/4.0)*height), width, newHeight);
            int[] frequencyByCol = collapseVerticallyByYellowCount(pixels, width, newHeight);
            ArrayList<Integer> interestingColumns = getColumnsWithRequiredBlackCount(frequencyByCol);
            ArrayList<Integer> colBounds = getColumnBounds(interestingColumns);
            if(colBounds.size() > 1) {
                int stoneWidth = colBounds.get(1) - colBounds.get(0);
                if (stoneCenters.get(1) - stoneCenters.get(0) > stoneWidth + 2) { // if the stones are split with something in between
                    Log.d("Stone based processing:", "CENTER");
                    return CENTER;
                } else if (stoneCenters.get(1) > 2 * stoneWidth + 4) { // if the right stone is far right and the left stone is in the middle
                    Log.d("Stone based processing:", "LEFT");
                    return LEFT;
                } else if (stoneCenters.get(1) > stoneWidth + 2) { // if the right stone is in the middle and the left stone is on the left
                    Log.d("Stone based processing:", "RIGHT");
                    return RIGHT;
                }
            }
        }
        return UNKNOWN;
    }

    public double getSkystonePositionInches(Bitmap bmp) {
        int width = bmp.getWidth(), height = bmp.getHeight();
        int newHeight = (int)(1.0/2.0 * height);
        int[] pixels = new int[width * height];
        bmp.getPixels(pixels, 0, width, 0, (int)((1.0/2.0)*height), width, newHeight);
        int[] columnFrequencies = collapseVerticallyByBlackCount(pixels, DESIRED_WIDTH, DESIRED_HEIGHT);
        ArrayList<Integer> columnsWithRequiredPercent = getColumnsWithRequiredBlackCount(columnFrequencies);
        ArrayList<Integer> skystoneBounds = getColumnBounds(columnsWithRequiredPercent);

        int skystoneWidth = Math.abs(skystoneBounds.get(1) - skystoneBounds.get(0));
        int distToCenter = (int)((skystoneBounds.get(1) - width/2.0) - (DESIRED_WIDTH / 2.0));
        double distToTravel = distToCenter * (7.0/skystoneWidth);
        return distToTravel;
    }

    /*
        this function will show the blue pixels in an image
     */
    public void showBlackPixels(int [] pixels, int height, int width, int colorToReplaceWith) {
        for (int c = 0; c < width; c++) {
            int numberOfBlackPixels = 0;
            for (int r = 0; r < height; r++) {
                int color = pixels[r * width + c];
                int[] rgba = {Color.red(color), Color.blue(color), Color.green(color), Color.alpha(color)};
                float[] hsv = new float[3];
                Color.colorToHSV(color, hsv);
                //check for blue
                if (checkIfBlack(hsv)) {
                    rgba[0] = 0;
                    rgba[1] = 250;
                    rgba[2] = 250;
                    pixels[r * width + c] = colorToReplaceWith;
                    numberOfBlackPixels++;
                }
            }
        }
    }


    public void showYellowPixels(int [] pixels, int height, int width, int colorToReplaceWith) {
        for (int c = 0; c < width; c++) {
            int numberOfYellowPixels = 0;
            for (int r = 0; r < height; r++) {
                int color = pixels[r * width + c];
                int[] rgba = {Color.red(color), Color.blue(color), Color.green(color), Color.alpha(color)};
                float[] hsv = new float[3];
                Color.colorToHSV(color, hsv);
                //check for blue
                if (checkIfYellow(hsv)) {
                    rgba[0] = 0;
                    rgba[1] = 250;
                    rgba[2] = 250;
                    pixels[r * width + c] = colorToReplaceWith;
                    numberOfYellowPixels++;
                }
            }
        }
    }

    public int [] collapseVerticallyByBlackCount(int [] pixels, int imageWidth, int imageHeight) {
        //collapse into a single, frequency based
        int [] toReturn = new int[imageWidth];
        for (int c = 0; c < imageWidth; c ++) {
            for (int r = 0; r < imageHeight; r ++) {
                int color = pixels[r * imageWidth + c];
                if(checkIfBlack(color)) {
                    toReturn[c]++;
                }
            }
        }
        return toReturn;
    }

    public int [] collapseVerticallyByYellowCount(int [] pixels, int imageWidth, int imageHeight) {
        //collapse into a single, frequency based
        int [] toReturn = new int[imageWidth];
        for(int c = 0; c < imageWidth; c ++){
            for(int r = 0; r < imageHeight; r ++){
                int color = pixels[r*imageWidth + c];
                if(checkIfYellow(color)){
                    toReturn[c] ++;
                }
            }
        }
        return toReturn;
    }

    public boolean skystoneNearCenter(int [] pixels, int imageWidth, int imageHeight, int targetWidth, int targetHeight) {
        boolean toReturn = false;
        int blackCount = 0, totalCount = 0;
        for(int c = (int)(imageWidth / 3.0); c < (2.0/3.0)*imageWidth; c++) {
            for(int r = (int)(imageHeight / 2.0); r < imageHeight; r++) {
                totalCount++;
                if(checkIfBlack(pixels[r*imageWidth + c])) blackCount++;
            }
        }
        if((double)(blackCount)/(double)(totalCount) > 0.50) toReturn = true;
        return toReturn;
    }

    public boolean checkIfBlack(float [] hsl) {
        if (hsl[0] > 0 && hsl[0] < 360) {
            if (hsl[2] < 0.35) {
                return true;
            }
        }
        return false;
    }

    public boolean checkIfYellow(float [] hsl) {
        if (hsl[0] > 0 || hsl[0] < 360) {
            if (hsl[1] > .64) {
                if (hsl[2] > .5) {
                    return true;
                }
            }
        }
        return false;
    }
    public boolean checkIfBlack(int color) {
        float [] hsl = new float[3];
        Color.colorToHSV(color,hsl);
        return checkIfBlack(hsl);
    }
    
    public boolean checkIfYellow(int color) {
        float [] hsl = new float[3];
        Color.colorToHSV(color,hsl);
        return checkIfYellow(hsl);
    }
}
