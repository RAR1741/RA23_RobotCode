//Credit: https://gist.github.com/pordonj/970b2c189cc6ee06388b3e2f12abcb72

package frc.robot.controls.controllers;

public class DPadButton {

    FilteredController controller;
    Direction direction;

    public DPadButton(FilteredController controller, Direction direction) {
        this.controller = controller;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        int dPadValue = controller.getPOV();
        return dPadValue == direction.direction;
        // return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
        //         || (dPadValue == (direction.direction + 315) % 360);
    }

}