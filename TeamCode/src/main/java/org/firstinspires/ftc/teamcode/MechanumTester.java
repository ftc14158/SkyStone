package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MechTest", group = "testing")

public class MechTest extends LinearOpMode {

    // motor objects

    private DcMotor drive_fl;   // actual name on robot configuration is drive-fl
    private DcMotor drive_fr;   // etc..
    private DcMotor drive_rl;
    private DcMotor drive_rr;
    private DcMotor arm1;

    // Previous values for change detection
    int PrevVD = 0; // Desired Robot Speed
    int PrevThetaD = 0; // Desired Angle
    int PrevVTheta = 0; // Desired Rotation Speed

    private void initializeDriveMotor(DcMotor m) {
        m.setPower(0);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     **  Mechanum drive function
     */
    private void MechDrive() {

        int VD; // Desired Robot Speed
        int ThetaD;  // Desired Angle
        int VTheta;  // Desired Rotation Speed

        int ThetaD45; // Desire Angle + 45o

        int V1; // Front Left motor
        int V2; // Front Right motor
        int V3; // Rear Left motor
        int V4; // Rear Right motor
        int LR; // Left/Right Command
        int FR; // Forward/Reverse Command
        int CCW; // Rotation command
        int RadioVD; // VD from joystick
        int RadioTh; // Theta from joystick

        //Capture joystick value
        // X and Y values need to be from -1000 to +1000
        // standard values are from -1 to +1

        LR = (int) ( gamepad1.left_stick_x * 1000 );
        FR = (int) (-gamepad1.left_stick_y * 1000 );  // invert value because negative is up on joystick
        CCW = (int) (gamepad1.right_stick_x * 1000 );

        // Compute distance of joystick from center position in any direction
        RadioVD = (int) ( ( Math.sqrt(LR * LR + FR * FR)) );

        // Compute angle of X-Y
        if( FR != 0 ) {
            float x, y;
            x = (float)LR;
            y = (float)FR;

            RadioTh = (int) ( Math.toDegrees( Math.atan( x/y ) ) ); // atan takes input * 1000 and returns angle in degrees * 10

            //   telemetry.addData("Theta", "x "+x+" y "+y+" radioth "+RadioTh );

            if( (LR >= 0) && (FR < 0) ) {
                RadioTh += 180;
            } else if( (LR < 0) && (FR < 0) ) {
                RadioTh -= 180;
            }
        } else if (LR > 0) {
            RadioTh = 90;
        } else if (LR < 0) {
            RadioTh = -90;
        } else {
            RadioTh = 0;
        }

        VD = RadioVD;
        ThetaD = RadioTh;
        VTheta = -CCW;

        // Uncomment below to check captured values in console
        //   telemetry.addData("Positions", "LR " + LR + " FR " + FR + " RadioVD " + RadioVD + " RadioTh " + RadioTh  );

        // To avoid unnecessary computation, evaluate formulas only if change occurred

        if( (VD != PrevVD) || (ThetaD != PrevThetaD) || (VTheta != PrevVTheta) ) {
            ThetaD45 = ThetaD + 45; // compute once angle + 45 for use in the 4 equations

            V1 = (int) ( (VD * Math.sin( Math.toRadians(ThetaD45) ) ) + VTheta ); // sin takes degrees and returns result * 1000
            V2 = (int) ( (VD * Math.cos( Math.toRadians(ThetaD45) ) ) - VTheta );
            V3 = (int) ( (VD * Math.cos( Math.toRadians(ThetaD45) ) ) + VTheta );
            V4 = (int) ( (VD * Math.sin( Math.toRadians(ThetaD45) ) ) - VTheta );

            // Uncomment below to view computed speeds in console

            telemetry.addData("Velocities", "V1 " +V1 + " V2 " + V2 + " V3 " + V3 + " V4 " + V4 );

            // Save for detecting change at next loop execution

            PrevVD = VD;
            PrevThetaD = ThetaD;
            PrevVTheta = VTheta;

            // Apply to local motors
            // v1 = fl, v2 = fr, v3 = rl, v4 = rr
            float fl, fr, rl, rr;

            fl = (float)V1 / 1000;
            fr = (float)V2 / 1000;
            rl = (float)V3 / 1000;
            rr = (float)V4 / 1000;

            drive_fl.setPower(fl);
            drive_rl.setPower(rl);
            drive_fr.setPower(-fr);
            drive_rr.setPower(-rr);

        } else { V1 = 0; V2 = 0; V3 = 0; V4 = 0; }


    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // Set the DCMotor objects to point to the four
        // drive motors for the Mechanum wheels

        drive_fl = hardwareMap.dcMotor.get("drive-fl");
        drive_fr = hardwareMap.dcMotor.get("drive-fr");
        drive_rl = hardwareMap.dcMotor.get("drive-rl");
        drive_rr = hardwareMap.dcMotor.get("drive-rr");
        //arm1 = hardwareMap.dcMotor.get("arm");

        // Set the motor operating modes and braking

        initializeDriveMotor(drive_fl);
        initializeDriveMotor(drive_fr);
        initializeDriveMotor(drive_rl);
        initializeDriveMotor(drive_rr);
        //initializeDriveMotor(arm1);

        // wait for operator to start opmode

        waitForStart();

        // Main opmode loop

        if (opModeIsActive()) {
            // Put run blocks here._
            double speed = 0, speed_delta = 0, apply_speed;

            while (opModeIsActive()) {


                // Put loop blocks here.
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.

      //          MechDrive();
      //          telemetry.update();

                // allow dpad up and down to alter the target speed by 10% up or down
                // one change per press
                // cant change speed when delta already in effect
                if (speed_delta == 0) {
                    if (gamepad1.dpad_up) speed_delta = 0.1;
                    if (gamepad1.dpad_down) speed_delta = -0.1;

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

                // allow back button held at same time to reverse the applied direction
                apply_speed = speed * (gamepad1.back ? -1 : 1);

                // if a b c or d pressed, apply speed to that motor

                drive_fl.setPower( gamepad1.a ? apply_speed : 0);
                drive_fr.setPower( gamepad1.b ? -apply_speed : 0);
                drive_rl.setPower( gamepad1.x ? apply_speed : 0);
                drive_rr.setPower( gamepad1.y ? -apply_speed : 0);


        /*
        speed = gamepad1.left_stick_y;

        if ( !(gamepad1.left_bumper || gamepad1.right_bumper)) {
          // straight ahead
          drive_fl.setPower(speed);
          drive_rl.setPower(speed);
          drive_fr.setPower(-speed);
          drive_rr.setPower(-speed);
        } else if (gamepad1.left_bumper) {
          // strafe left - to go left, left side wheels drive toward each other
          // and right side the opposite
          drive_fl.setPower(-speed);   // front left going backwards towards back left
          drive_rl.setPower(speed);
          drive_fr.setPower(-speed);
          drive_rr.setPower(speed);   // rear right going backwards away from front right
        // strafe right - right side toward each other
        } else if (gamepad1.right_bumper) {
          drive_fl.setPower(speed);
          drive_rl.setPower(-speed);
          drive_fr.setPower(speed);
          drive_rr.setPower(-speed);
        }

        // operate Arm
      //  arm1.setPower( gamepad1.right_stick_y);
        */


                //telemetry.addData("Left Pow", left_driveAsDcMotor.getPower());
                //telemetry.addData("Right Pow", right_driveAsDcMotor.getPower());
                //telemetry.update();
            }
        }
    }
}
