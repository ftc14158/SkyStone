package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MechanumDrive", group = "manual")

public class MechanumDrive extends LinearOpMode {
    // motor objects


    private Mechanum mech;

    private DcMotor arm1;
    private DcMotor arm2;
    private Servo grab;


    private Tracking track;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // Set up IMU tracking

        track = new Tracking( hardwareMap.get(BNO055IMU.class, "imu") );

        track.initialize();

        // Set the DCMotor objects to point to the four
        // drive motors for the Mechanum wheels

        mech = new Mechanum(
                hardwareMap.dcMotor.get("drive-fl"),
                hardwareMap.dcMotor.get("drive-fr"),
                hardwareMap.dcMotor.get("drive-rl"),
                hardwareMap.dcMotor.get("drive-rr")
        );

        mech.InitializeMotors();

        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        grab = hardwareMap.servo.get("grab");

        arm1.setPower(0);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setPower(0);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set the DCMotor objects to point to the four
        // drive motors for the Mechanum wheels


        waitForStart();

        // Main opmode loop

        if (opModeIsActive()) {
            // Put run blocks here._
            double speed = 0, speed_delta = 0, apply_speed;

            double servoPos = grab.getPosition();

            track.startTracking();

            while (opModeIsActive()) {

                int LR; // Left/Right Command
                int FR; // Forward/Reverse Command
                int CCW; // Rotation command

                //Capture joystick value
                // X and Y values need to be from -1000 to +1000
                // standard values are from -1 to +1

                LR = (int) ( gamepad1.left_stick_x * 1000 );
                FR = (int) (-gamepad1.left_stick_y * 1000 );  // invert value because negative is up on joystick
                CCW = (int) (gamepad1.right_stick_x * 1000 );

                mech.MechDrive(LR, FR, CCW);

                // allow dpad up and down to alter the target speed by 10% up or down
                // one change per press
                // cant change speed when delta already in effect
                if (speed_delta == 0) {
                    if (gamepad1.dpad_up) speed_delta = 0.025;
                    if (gamepad1.dpad_down) speed_delta = -0.025;

                    // apply the speed delta.. it will be ignored in subsequent loops until
                    // dpad is released
                    if (speed_delta != 0) {
                        speed += speed_delta;
                        if (speed < 0) speed = 0;
                        if (speed > 1) speed = 1;

                        telemetry.addData("Speed changed", speed);
                        telemetry.update();
                    }


                } else {
                    if ( !(gamepad1.dpad_up || gamepad1.dpad_down) ) speed_delta = 0;
                }

                boolean arm1up = gamepad1.a;
                boolean arm1down = gamepad1.b;

                if (arm1up) {
                    arm1.setPower( speed );
                } else if (arm1down) {
                    arm1.setPower( -speed );
                } else {
                    arm1.setPower( 0 );
                }

                boolean arm2up = gamepad1.x;
                boolean arm2down = gamepad1.y;


                if (arm2up) {
                    arm2.setPower( speed );
                } else if (arm2down) {
                    arm2.setPower( -speed );
                } else {
                    arm2.setPower( 0 );
                }

                if ( gamepad1.left_bumper ) {
                    servoPos += (1/90);
                    grab.setPosition(servoPos);
                } else if (gamepad1.right_bumper) {
                    servoPos -= (1/90);
                    grab.setPosition(servoPos);
                }

            }
        }
    }
}
