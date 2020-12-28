package Misc;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * Author: Software Team
 * Date: 10/6/2020
 *
 * Takes the output from the Control Hub, and finds the output, which is then put in a json file
 */

public class ControlHubLogParser {

    public static final String PATH = "jsonFile.json";

    public static void main(String[] args) throws Exception {

        File file = new File("TeamCode/src/main/java/Misc/LogFile.txt");
        System.out.println(file.getAbsolutePath());
        Scanner input = new Scanner(file);

        // get valid lines from the file
        ArrayList<String> validLines = new ArrayList<>();
        while(input.hasNext()) {

            // get the line from the file
            String line = input.nextLine();

            // make sure it's on a line with the json object
            if (line.contains("Object") && line.contains("{")) {
                line = line.substring(line.indexOf("{"));
                validLines.add(line);
            }
        }

        // only add the final line in the list
        PrintWriter writer = new PrintWriter("TeamCode/src/main/java/Autonomous/OpModes/JSONFiles/" + PATH, "UTF-8");
        writer.println(validLines.get(validLines.size() - 1));
        writer.close();

    }

}
