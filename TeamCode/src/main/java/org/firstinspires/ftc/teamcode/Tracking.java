package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Tracking {

    // The Tracking sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Position curPosition;

    public Tracking( BNO055IMU imuInterface ) {
        imu = imuInterface;
    }

    /***
     * Initialize the Tracking with calib data file if available
     *
     * @return true if succeeds
     */
    public boolean initialize() {
        // Set up the parameters with which we will use our Tracking. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the Tracking. We expect the Tracking to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit Tracking",
        // and named "imu".

        return imu.initialize(parameters);

    }

    /***
     * Call at start of main run to track acceleration for dead reckoning
     */

    public void startTracking() {
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /***
     * Call this regularly within loop to get updated tracking information
     */
    public void readTracking() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // gravity  = imu.getGravity();
        curPosition = imu.getPosition();
    }

    public float heading() {
        return angles.firstAngle;
    }

    public Position position() {
        return curPosition;
    }



}
