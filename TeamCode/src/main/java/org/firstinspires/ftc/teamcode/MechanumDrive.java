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

    private GamepadSnapshot gp1;
    private GamepadSnapshot gp2;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // Set up IMU tracking

//        track = new Tracking( hardwareMap.get(BNO055IMU.class, "imu") );

 //       track.initialize();

        // Set the DCMotor objects to point to the four
        // drive motors for the Mechanum wheels

        mech = new Mechanum(
                hardwareMap.dcMotor.get("drive-fl"),
                hardwareMap.dcMotor.get("drive-fr"),
                hardwareMap.dcMotor.get("drive-rl"),
                hardwareMap.dcMotor.get("drive-rr"),
                DcMotorSimple.Direction.REVERSE
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

        // Set up to snapshot gamepad1
        gp1 = new GamepadSnapshot(gamepad1);
        gp2 = new GamepadSnapshot(gamepad2);


        waitForStart();

        // Main opmode loop

        if (opModeIsActive()) {
            // Put run blocks here._
            double speed = 0, speed_delta = 0, apply_speed;

            double servoPos = grab.getPosition();
            double servoInc = 0.005;

        //    track.startTracking();

            while (opModeIsActive()) {

                gp1.snapshot();
                gp2.snapshot();

                //telemetry.addData("Left x, y, angle", "%1.3f %1.3f %3.2f", gp.value.left_stick_x, gp.value.left_stick_y, gp.getLeftStickAngle() );
                //telemetry.addData("Left Power", gp.getLeftStickPower() );


                mech.MechSet( gp1.getLeftStickAngle(), gp1.getLeftStickPower(), gp1.value.right_stick_x );

                //mech.MechDrive(LR, FR, CCW);

                // allow dpad up and down to alter the target speed by 10% up or down
                // one change per press
                // cant change speed when delta already in effect


/*                if (gp.dpadUpClicked()) speed_delta = 0.025;
                if (gp.dpadDownClicked()) speed_delta = -0.025;
                if (gp.dpadUpClicked() || gp.dpadDownClicked()) {
                    speed += 0.025 * (gp.dpadUpClicked() ? 1 : -1);
                    if (speed < 0) speed = 0;
                    if (speed > 1) speed = 1;

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

                boolean arm2up = gp.value.x;
                boolean arm2down = gp.value.y;


                if (arm2up) {
                    arm2.setPower( speed );
                } else if (arm2down) {
                    arm2.setPower( -speed );
                } else {
                    arm2.setPower( 0 );
                }
*/

                // control arm mototrs with gamepad2

                arm1.setPower( -gp2.logSlope(gp2.value.left_stick_y / 8) );
                arm2.setPower( -gp2.logSlope( gp2.value.right_stick_y / 8) );

                double servoChange = 0;
                if ( gp2.value.left_bumper && (servoPos < 1) )  servoChange = servoInc;
                if ( gp2.value.right_bumper && (servoPos > 0.5) ) servoChange = -servoInc;

                if ( servoChange != 0 ) {
                    servoPos += servoChange;

                    grab.setPosition(servoPos);
                }

             //   track.readTracking();

                telemetry.addData("Speed setting:", speed);
                telemetry.addData("Servo set /  position:", servoPos + " / " + grab.getPosition() );
                telemetry.addData( "fr speed", mech.drive_fl.getPower() );
        //        telemetry.addData("Heading:", track.heading());
        //        telemetry.addData("Position", track.curPosition.x +"," + track.curPosition.y + "," + track.curPosition.z );

                telemetry.update();

            }
        }
    }
}
