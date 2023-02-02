package frc.robot.autonomous.parsing;

import java.util.List;

public class AutoInstruction {
    public String type;
    public Unit unit;
    public Double amount;
    public List<Double> args;

    public enum Unit {
        SECONDS, MILLISECONDS, ENCODER_TICKS, ROTATIONS, INCHES, FEET, CURRENT, DEGREES, INVALID
    }

    public AutoInstruction(String type, List<Double> args) {
        this.type = type;
        this.args = args;
    }

    public AutoInstruction(String type, Unit unit, Double amount, List<Double> args) {
        this.type = type;
        this.unit = unit;
        this.amount = amount;
        this.args = args;
    }

    public static Unit parseUnit(String in) {
        return Unit.valueOf(in);
    }
}