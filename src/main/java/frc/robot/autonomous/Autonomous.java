package frc.robot.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.autonomous.parsing.*;
import frc.robot.logging.*;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Autonomous implements Loggable {

    private static final double TICKS_PER_ROTATION = 16750; // TODO: Update value for 2023 robot
    private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / (6 * Math.PI); // TODO: Update formula for 2023 robot
    private int step;
    private double start;
    private double navxStart;

    private AHRS gyro;

    private SwerveDrive drive;

    private AutoParser parser;
    private LoggableTimer timer;


    /**
     * Creates an Autonomous from the specified file
     * 
     * @param file The location of the file to parse
     */
    public Autonomous(String file, AHRS gyro, SwerveDrive drive) {
        this.drive = drive;
        this.gyro = gyro;

        parseFile(file);
    }

    public void parseFile(String file) {
        step = -1;
        timer = new LoggableTimer();
        parser = new AutoParser(file);
        for(AutoInstruction ai : parser.INSTRUCTIONS) { //Debug prints
            System.out.println("Block " + (parser.INSTRUCTIONS.indexOf(ai) + 1));
            System.out.println("Type: " + ai.type);
            System.out.println("Unit: " + ai.unit);
            System.out.println("Amount: " + ai.amount);
            System.out.println("Args:");
            for(double i : ai.args) {
                System.out.println("\t" + i);
            }
            System.out.println();
        }
    }

    public static AutoInstruction.Unit parseUnit(String in) {
        return AutoInstruction.Unit.valueOf(in);
    }

    public void run() {
        if (step == -1) {
            reset();
        }
        if (parser.INSTRUCTIONS.size() == step) {
            drive.drive(0, 0, 0, false);
            return;
        }
        AutoInstruction ai = parser.INSTRUCTIONS.get(step);

        switch (ai.type) {
            case "drive":
                drive(ai);
                break;

            case "turnDeg":
                turnDegrees(ai);
                break;

            case "wait":
                wait(ai);
                break;

            default:
                System.out.println("Invalid Command");
                reset();
                break;
        }
    }

    //TODO: THIS IS PROBABLY VERY BROKEY
    //---//
    private void drive(AutoInstruction ai) {
        AutoInstruction.Unit u = ai.unit;
        // ai args:
        // 0: xPower
        // 1: yPower
        // 2: rotation Power
        // 3: field oriented (1 if true, 0 if false)
        if (u.equals(AutoInstruction.Unit.SECONDS) || u.equals(AutoInstruction.Unit.MILLISECONDS)) {
            // amount: (milli)seconds to drive
            if (driveTime(ai.args.get(0), ai.args.get(1), ai.args.get(2), (u.equals(AutoInstruction.Unit.SECONDS) ? ai.amount : ai.amount / 1000.0), ai.args.get(3) == 1)) {
                reset();
            }
        } else if (u.equals(AutoInstruction.Unit.ENCODER_TICKS) || u.equals(AutoInstruction.Unit.ROTATIONS)) {
            // amount: rotations/encoder ticks to drive
            if (driveDistance(ai.args.get(0), ai.args.get(1), ai.args.get(2), (u.equals(AutoInstruction.Unit.ENCODER_TICKS) ? ai.amount : ai.amount * TICKS_PER_ROTATION), ai.args.get(3) == 1)) {
                reset();
            }
        } else if (u.equals(AutoInstruction.Unit.FEET) || u.equals(AutoInstruction.Unit.INCHES)) {
            // amount: feet/inches to drive
            if (driveDistance(ai.args.get(0), ai.args.get(0), ai.args.get(2), (u.equals(AutoInstruction.Unit.INCHES) ? ai.amount * TICKS_PER_INCH : (ai.amount * TICKS_PER_INCH) * 12), ai.args.get(3) == 1)) {
                reset();
            }
        }
    }

    private boolean driveDistance(double leftPower, double rightPower, double turnPower, double distance, boolean field) {
        if (Math.abs(drive.getEncoder() - start) < distance) {
            drive.drive(leftPower, rightPower, turnPower, field);
        } else {
            return true;
        }
        return false;
    }

    private boolean driveTime(double leftPower, double rightPower, double turnPower, double time, boolean field) {
        if (timer.get() < time) {
            drive.drive(leftPower, rightPower, turnPower, field);
        } else {
            return true;
        }
        return false;
    }

    private boolean rotateDegrees(double turnPower, double deg) {
        System.out.println(getAngle());
        if (Math.abs(getAngle() - navxStart - deg) < 2) {
            return true;
        } else {
            drive.drive(0, 0, turnPower, false);
            return false;
        }
    }

    public void turnDegrees(AutoInstruction ai) {
        // ai args:
        // 0: leftPower
        // 1: rightPower
        // amount: degrees to turn
        if (rotateDegrees(ai.args.get(0), ai.amount)) {
            drive.drive(0, 0, 0, false); // Stop turning
            reset();
        }
    }
    //---//

    private void wait(AutoInstruction ai) {
        if (ai.unit.equals(AutoInstruction.Unit.MILLISECONDS)) {
            if (timer.get() / 1000 >= ai.amount) {
                reset();
            }
        } else {
            if (timer.get() >= ai.amount) {
                reset();
            }
        }
    }

    private void reset() {
        drive.drive(0, 0, 0, true);
        step++;
        navxStart = getAngle();
        start = drive.getEncoder();
        timer.reset();
        timer.start();
    }

    private double getAngle() {
        return gyro.getAngle();
    }

    public static String getAutoPath(String name) {
        return Filesystem.getDeployDirectory().listFiles((dir, filename) -> {
            return filename.contentEquals("autos");
        })[0].listFiles((dir, filename) -> {
            return filename.contentEquals(name);
        })[0].getAbsolutePath();
    }

    @Override
    public void logHeaders(Logger logger) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void logData(Logger logger) {
        // TODO Auto-generated method stub
        
    }
}
