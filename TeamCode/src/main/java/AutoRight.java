package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutoRight", group = "")

public class AutoRight extends LinearOpMode {


    private Mechanum mech;

    private DcMotor arm1;



    private void initializeDriveMotor(DcMotor m) {
        m.setPower(0);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

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


        // wait for operator to start opmode

        waitForStart();

        // Main opmode loop

        if (opModeIsActive()) {
            // Put run blocks here.

            while (opModeIsActive()) {
                float speed;

                // Put loop blocks here.

                mech.MechSet(90, .5, 0);
                sleep(4000);

                mech.MechSet(0, 0, 0);

                while (opModeIsActive()) {
                    sleep(500);
                }

            }
        }
    }
}
