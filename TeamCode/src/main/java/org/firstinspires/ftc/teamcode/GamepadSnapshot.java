package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

/***
 * Read and retain values from a gamepad with calculations
 */
public class GamepadSnapshot {


    public class GamepadValues implements Cloneable {
        /**
         * left analog stick horizontal axis
         */
        public float left_stick_x = 0f;

        /**
         * left analog stick vertical axis
         */
        public float left_stick_y = 0f;

        /**
         * right analog stick horizontal axis
         */
        public float right_stick_x = 0f;

        /**
         * right analog stick vertical axis
         */
        public float right_stick_y = 0f;

        /**
         * dpad up
         */
        public boolean dpad_up = false;

        /**
         * dpad down
         */
        public boolean dpad_down = false;

        /**
         * dpad left
         */
        public boolean dpad_left = false;

        /**
         * dpad right
         */
        public boolean dpad_right = false;

        /**
         * button a
         */
        public boolean a = false;

        /**
         * button b
         */
        public boolean b = false;

        /**
         * button x
         */
        public boolean x = false;

        /**
         * button y
         */
        public boolean y = false;

        /**
         * button guide - often the large button in the middle of the controller. The OS may
         * capture this button before it is sent to the app; in which case you'll never
         * receive it.
         */
        public boolean guide = false;

        /**
         * button start
         */
        public boolean start = false;

        /**
         * button back
         */
        public boolean back = false;

        /**
         * button left bumper
         */
        public boolean left_bumper = false;

        /**
         * button right bumper
         */
        public boolean right_bumper = false;

        /**
         * left stick button
         */
        public boolean left_stick_button = false;

        /**
         * right stick button
         */
        public boolean right_stick_button = false;

        /**
         * left trigger
         */
        public float left_trigger = 0f;

        /**
         * right trigger
         */
        public float right_trigger = 0f;

        @Override
        protected Object clone() throws CloneNotSupportedException {
            return super.clone();
        }
    }

    private Gamepad gamepad;
    public GamepadValues value;
    private GamepadValues previousV;

    private boolean leftJoyCalced = false, rightJoyCalced = false;
    private double leftStickAngle, leftStickPower;
    private double rightStickAngle, rightStickPower;


    /*
        The following are true only after first snapshot when
        down, and stay false after under button is released
     */

    public GamepadSnapshot(Gamepad gp) {
        value = new GamepadValues();
        previousV = new GamepadValues();

        gamepad = gp;
    }

    /**
     * Take a snapshot of current gamepad values
     *
     * Subsequent caller code can use this snapshot in this code to
     * avoid having values change mid-processing
     */
    public void snapshot()  {

        // Take copy of current state first
        try {
            previousV = (GamepadValues) value.clone();
        } catch(CloneNotSupportedException e) {
            RobotLog.e("GamepadSnapshot state clone failure");
            throw new IllegalStateException("GamepadSnapshot state clone failure");
        }

        /**
         * left analog stick horizontal axis
         */
        value.left_stick_x = gamepad.left_stick_x;
        value.left_stick_y = gamepad.left_stick_y;
        value.right_stick_x = gamepad.right_stick_x;
        value.right_stick_y = gamepad.right_stick_y;
        value.dpad_up = gamepad.dpad_up;
        value.dpad_down = gamepad.dpad_down;
        value.dpad_left = gamepad.dpad_left;
        value.dpad_right = gamepad.dpad_right;
        value.a = gamepad.a;
        value.b = gamepad.b;
        value.x = gamepad.x;
        value.y = gamepad.y;
        value.guide = gamepad.guide;
        value.start = gamepad.start;
        value.back = gamepad.back;
        value.left_bumper = gamepad.left_bumper;
        value.right_bumper = gamepad.right_bumper;
        value.left_stick_button = gamepad.left_stick_button;
        value.right_stick_button = gamepad.right_stick_button;
        value.left_trigger = gamepad.left_trigger;
        value.right_trigger = gamepad.right_trigger;

        // Reset joystick calculation flags
        leftJoyCalced = false;
        rightJoyCalced = false;
    }

    private static double[] calcStick(float x, float y) {
        double angle, power;

        y = -y;    // down is 1 on stick

        if (y != 0) {
            angle =  Math.toDegrees(Math.atan( x / y ) );

            if( (x >= 0) && (y < 0) ) {
                angle += 180;
            } else if( (x < 0) && (y < 0) ) {
                angle -= 180;
            }
        } else if (x > 0) {   // strafing pure right
            angle = 90;
        } else if (x < 0) {   // strafing pure left
            angle = -90;
        } else {
            angle = 0;       // no movement
        }

        // Power can be > 1 at extremes, limit to 1 max

        power = Math.min( Math.sqrt(x * x + y * y), 1.0);

        return new double[] { angle, power };
    }

    private boolean leftCalced() {
        if (!leftJoyCalced) {
            double[] values = calcStick( value.left_stick_x, value.left_stick_y);
            leftStickAngle = values[0];
            leftStickPower = values[1];
            leftJoyCalced = true;
        }
        return leftJoyCalced;
    }

    private boolean rightCalced() {
        if (!rightJoyCalced) {
            double[] values = calcStick( value.right_stick_x, value.right_stick_y);
            rightStickAngle = values[0];
            rightStickPower = values[1];
            rightJoyCalced = true;
        }
        return rightJoyCalced;
    }

    public double getLeftStickAngle() {
        return leftCalced() ? leftStickAngle : 0;
    }

    public double getLeftStickPower() {
        return leftCalced() ? leftStickPower : 0;
    }

    public double getRightStickAngle() {
        return rightCalced() ? rightStickAngle : 0;
    }

    public double getRightStickPower() {
        return rightCalced() ? rightStickPower : 0;
    }

    public boolean dpadUpClicked() {
        return value.dpad_up && !previousV.dpad_up;
    }

    public boolean dpadDownClicked() {
        return value.dpad_down && !previousV.dpad_down;
    }

    public boolean dpadLeftClicked() {
        return value.dpad_left && !previousV.dpad_left;
    }

    public boolean dpadRightClicked() {
        return value.dpad_right && !previousV.dpad_right;
    }

    public boolean leftBumperClicked() {
        return value.left_bumper && !previousV.left_bumper;
    }

    public boolean rightBumperClicked() {
        return value.right_bumper && !previousV.right_bumper;
    }

    // make 0 -> 1 value ramp up more slowly for better joystickcontrol

    public double logSlope( double x ) {
        if (x >= 0) {
            return (1 - Math.log10(     1 + ( (1- x) * 9) ) );
        } else {
            return - (1 - Math.log10(     1 + ( (1 + x) * 9) ) );
        }
    }
}


