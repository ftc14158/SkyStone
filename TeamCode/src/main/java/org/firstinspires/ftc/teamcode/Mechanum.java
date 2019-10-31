package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.channels.MulticastChannel;

public class Mechanum  {


    public DcMotor drive_fl;   // actual name on robot configuration is drive-fl
    public DcMotor drive_fr;   // etc..
    public DcMotor drive_rl;
    public DcMotor drive_rr;

    // Previous values for change detection
    int PrevVD = 0; // Desired Robot Speed
    int PrevThetaD = 0; // Desired Angle
    int PrevVTheta = 0; // Desired Rotation Speed

    public Mechanum(DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr) {
        drive_fl = fl;
        drive_fr = fr;
        drive_rl = rl;
        drive_rr = rr;
    }

    private void initializeDriveMotor(DcMotor m) {
        m.setPower(0);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void InitializeMotors() {
        for(DcMotor m : new DcMotor[] {drive_fl, drive_fr, drive_rl, drive_rr} )
            initializeDriveMotor(m);

        // Previous values for change detection
        int PrevVD = 0; // Desired Robot Speed
        int PrevThetaD = 0; // Desired Angle
        int PrevVTheta = 0; // Desired Rotation Speed
    }

    /**
     **  Mechanum drive function
     */
    public void MechDrive(
            int LR, // Left/Right Command
            int FR, // Forward/Reverse Command
            int CCW // Rotation command
            ) {


        int VD; // Desired Robot Speed
        int ThetaD;  // Desired Angle
        int VTheta;  // Desired Rotation Speed

        int ThetaD45; // Desire Angle + 45o

        int V1; // Front Left motor
        int V2; // Front Right motor
        int V3; // Rear Left motor
        int V4; // Rear Right motor
        int RadioVD; // VD from joystick
        int RadioTh; // Theta from joystick

        //Capture joystick value
        // X and Y values need to be from -1000 to +1000
        // standard values are from -1 to +1

      //  LR = (int) ( gamepad1.left_stick_x * 1000 );
      //  FR = (int) (-gamepad1.left_stick_y * 1000 );  // invert value because negative is up on joystick
      //  CCW = (int) (gamepad1.right_stick_x * 1000 );

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

            //telemetry.addData("Velocities", "V1 " +V1 + " V2 " + V2 + " V3 " + V3 + " V4 " + V4 );

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


}
