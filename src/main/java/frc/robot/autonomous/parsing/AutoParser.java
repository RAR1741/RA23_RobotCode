// Version 2.1
package frc.robot.autonomous.parsing;

import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;

public class AutoParser {

    private final ArrayList<String> LINES = new ArrayList<>();
    public ArrayList<AutoInstruction> INSTRUCTIONS = new ArrayList<>();

    private final String path;

    /**
     * Creates a parser to convert json data into data java classes can read
     *
     * @param path the path to the json instruction file
     */

    public AutoParser(String path) {
        this.path = path;
        parse();
    }

    public void parse() {
        if(LINES.isEmpty()) {
            Scanner fileReader = null;
            try {
                fileReader = new Scanner(new File(path));
            } catch(FileNotFoundException e) {
                e.printStackTrace();
            }

            assert fileReader != null;

            while(fileReader.hasNextLine()) {
                LINES.add(fileReader.nextLine());
            }

            for(int i = 0; i < LINES.size(); i++) {
                LINES.set(i, LINES.get(i).replace(" ", ""));
                LINES.set(i, LINES.get(i).replace(",", ""));
                LINES.set(i, LINES.get(i).replace("\"", ""));
                LINES.set(i, LINES.get(i).replace(":", "="));
            }

            for(int i = 0; i < LINES.size(); i++) {
                String type = null;
                String unit = null;
                Double amount = null;
                ArrayList<Double> args = new ArrayList<>();
                if(LINES.get(i).contains("{")) {
                    i++;
                    if(LINES.get(i).contains("type")) {
                        type = LINES.get(i).substring(5);
                        i++;
                    }
                    if(LINES.get(i).contains("unit")) {
                        unit = LINES.get(i).substring(5);
                    }
                    i++;
                    if(LINES.get(i).contains("amount")) {
                        amount = Double.parseDouble(LINES.get(i).substring(7));
                        i++;
                    }
                    if(LINES.get(i).contains("args")) {
                        i++;
                        while(!LINES.get(i).contains("]")) {
                            if(!LINES.get(i).equals("[")) {
                                args.add(Double.parseDouble(LINES.get(i)));
                            }
                            i++;
                        }
                    } else { i--; }
                    if (i < LINES.size() - 1) { i++; }
                    if(LINES.get(i).contains("}")) {
                        INSTRUCTIONS.add(unit == null ? new AutoInstruction(type, args) : new AutoInstruction(type, AutoInstruction.parseUnit(unit), amount, args));
                    }
                }
            }
        }
    }

}