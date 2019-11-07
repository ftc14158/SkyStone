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

    public DcMotorSimple.Direction globalMotorDirection;


    public Mechanum(DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr, DcMotorSimple.Direction dir) {
        drive_fl = fl;
        drive_fr = fr;
        drive_rl = rl;
        drive_rr = rr;
        globalMotorDirection = dir;
    }

    private void initializeDriveMotor(DcMotor m, DcMotorSimple.Direction dir) {
        m.setPower(0);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setDirection(dir);
    }

    public void InitializeMotors() {
        for(DcMotor m : new DcMotor[] {drive_fl, drive_fr, drive_rl, drive_rr} )
            initializeDriveMotor(m, globalMotorDirection);

    }


    /***
     * Apply power to motors to set given direction and speed
     *
     * @param angle     Desired angle relative to forward: 0 = straight ahead, 90 = right, -90 or 270 = left, 180 = back
     * @param speed     Speed required from 0 to 1
     * @param rotation  Clockwise (towards right) rotational speed to apply, from 0 to 1
     */
    public void MechSet(double angle, double speed, float rotation) {

        // add 45 to desired angle to allow for mechanum wheel roller angle
        double ThetaD45 = angle + 45; // compute once angle + 45 for use in the 4 equations

        // Speeds to apply to motors (range 0 to 1)
        double fl, fr, rl, rr;


        fl =  ( (Math.sin( Math.toRadians(ThetaD45) ) ) ); // sin takes degrees and returns result * 1000
        fr = ( (Math.cos( Math.toRadians(ThetaD45) ) )  );
        rl =  ( (Math.cos( Math.toRadians(ThetaD45) ) )  );
        rr =  ( (Math.sin( Math.toRadians(ThetaD45) ) )  );

        // Add scale factor to achieve maximum power for any given distribution across drive wheels
        double sc = 1/ Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max( Math.abs(rl), Math.abs(rr) ));

        speed = speed * sc;

        // scale speeds to desired power

        fl = fl * speed;
        fr = fr * speed;
        rl = rl * speed;
        rr = rr * speed;

//        System.out.format("%5.5f    %5.5f\n", fl, fr);
//        System.out.format("%5.5f    %5.5f\n", rl, rr);

        // add requested rotation power with scaling if > 1

        sc = 1/ Math.max(Math.max(Math.abs(fl+rotation), Math.abs(fr-rotation)), Math.max( Math.abs(rl+rotation), Math.abs(rr-rotation) ));
        if (sc > 1) sc = 1;

  //      System.out.format("Scale %f\n", sc);

        fl = (fl + rotation) * sc;
        fr = (fr - rotation) * sc;
        rl = (rl + rotation) * sc;
        rr = (rr - rotation) * sc;

    //    System.out.format("%5.5f    %5.5f\n", fl, fr);
    //    System.out.format("%5.5f    %5.5f\n", rl, rr);

        drive_fl.setPower(fl);
        drive_rl.setPower(rl);
        drive_fr.setPower(-fr);
        drive_rr.setPower(-rr);


    }


}
