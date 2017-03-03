package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by corkransp on 1/6/17.
 */
@Autonomous(name="VuforiaOp", group="Autonomous")
public class VuforiaOp extends LinearOpMode {

    public DcMotorController right;
    public DcMotorController left;
    public DcMotor rfront;
    public DcMotor rback;
    public DcMotor lfront;
    public DcMotor lback;

    public ServoController servos;
    public Servo button;
    public final double MAX_POS = 1;
    public final double MIN_POS = 0.05;

    public ColorSensor rgb;

    public ElapsedTime timer;

    int phase;
    boolean flag;

    VuforiaTrackables beacons;
    VuforiaTrackable target;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "Ac3qG8r/////AAAAGXLdUcegR0CbgUBNrYsDw9M9AMaWE0WvgNmqiOaRlRUTw1cfI/jkGK3p5O+Tn2GzczmXGHXAJX7xJJgXiKf0onvEgr1++E0YCX5HALzMQFLls4mnZPpmrJPEGvNjJOyW83QXNfUwD8KlDP8t4Ge8Xzi/i2h34nC9DQvy5nqsoz3AJxZPqesQy1RaJmey6uTuJ5vHBDgDx5B3ToIP5fZgHwq3sX1u/3+HQb9BtTPvASZT/UkuMxZsXTf1MLajgJEiTWa9wFysqb684AHqFgDay0gYKWAECifCqSIJ9OH/U4SFtCaJa6dkxNZj51x6cdhXSPsZo+Y4vsDtzoV+V1kg57dTLSZK6up3DmoY10rK6FCM";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer v = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        beacons = v.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        right = hardwareMap.dcMotorController.get("aux");
        left = hardwareMap.dcMotorController.get("drive");

        lfront = hardwareMap.dcMotor.get("left");
        lback = hardwareMap.dcMotor.get("lback");
        rfront = hardwareMap.dcMotor.get("right");
        rback = hardwareMap.dcMotor.get("rback");

        servos = hardwareMap.servoController.get("servo");
        button = hardwareMap.servo.get("button");

        rgb = hardwareMap.colorSensor.get("rgb");

        lfront.setDirection(DcMotor.Direction.REVERSE);
        lback.setDirection(DcMotor.Direction.REVERSE);
        rfront.setDirection(DcMotor.Direction.FORWARD);
        rback.setDirection(DcMotor.Direction.FORWARD);

        lfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        lfront.setPower(0);
        lback.setPower(0);
        rfront.setPower(0);
        rback.setPower(0);

        button.setPosition(MIN_POS);

        waitForStart();

        beacons.activate();
        timer = new ElapsedTime();
        timer.reset();;

        phase = 0;
        target = null;
        OpenGLMatrix pose;
        VectorF translation;
        flag = false; // need a better name...

        while(opModeIsActive())
        {
            pose = null;

            telemetry.addData("Time: ", timer.time());
            telemetry.addData("Phase: ", phase);
            switch(phase)
            {
                //phase 0 - push yoga ball
                case 0:
                    if(timer.time() < 2.25) {
                        setPower(1);
                    }
                    else {
                        setPower(0);
                        phase++;
                        timer.reset();
                    }
                    break;
                // Phase 1 - locate beacon
                case 1:
                    //Pulse movement - spend .1 seconds turning, then pause for .2
                    if(timer.time() % 0.3 > 0.1) {
                        setSpinPower(0);
                    }
                    else {
                        setSpinPower(-.25);
                    }
                    pose = getPose("Tools", beacons);
                    if(pose != null) {
                        setSpinPower(0);
                        phase++;
                        timer.reset();
                    }
                    break;
                // Phase 2 - Fine-tune aim
                case 2:
                    translation = getTranslation("Tools", beacons);
                    if(translation != null) {
                        double degrees = getDegrees(translation);
                        double deltaDeg = getDeltaDegrees(degrees);
                        //Waiting for confirmation on aim
                        if(flag && deltaDeg < 1) {
                            //Confirmed!
                            if(timer.time() > .25) {
                                phase++;
                                flag = false;
                                timer.reset();
                            }
                        }
                        //We think we're aimed aux, but we might over/under-shoot /w momentum
                        else if(!flag && deltaDeg < 1) {
                            setSpinPower(0);
                            flag = true;
                            timer.reset();
                        }
                        //Pulse movement - spend .05 seconds turning, then pause for .15
                        else if(timer.time() % .2 > .15) {
                            if(degrees > 0)
                                setSpinPower(-.1);
                            else
                                setSpinPower(.1);
                            flag = false;
                        }
                        else
                            setSpinPower(0);
                    }
                    break;
                // Phase 3 - Move to beacon
                case 3:
                    translation = getTranslation("Tools", beacons);
                    if(translation != null) {
                        double z = Math.abs(translation.get(2));
                        telemetry.addData("Z: ", z);

                        double pow = 0;
                        //In front of beacon
                        if (z < 250) {
                            phase++;
                            timer.reset();
                        } else { //Not there yet
                            pow = .5 * (z / 2000);

                        }
                        //Are we going off-path?
                        double d = getDegrees(translation);
                        double deltaD = getDeltaDegrees(d);
                        if(deltaD > 5) {
                            if (d > 0)
                                setTurnPower(-pow, .8);
                            else
                                setTurnPower(pow, .8);
                        }
                        else {
                            setPower(pow);
                        }
                    }
                    //Uh-oh, we lost track of the image...
                    else {
                        setPower(0);
                        //TODO: Have some recovery code.  Make the robot back up, spin, whatever it needs to do
                    }
                    break;
                case 4:
                    //Debug
                    /*pose = getPose("Tools", beacons);
                    if(pose != null) {
                        translation = getTranslation("Tools", beacons);
                        telemetry.addData("translation: ", translation.toString());
                        double degrees = getDegrees(translation);
                        double deltaDeg = getDeltaDegrees(degrees);
                        telemetry.addData("degrees: ", degrees);
                        telemetry.addData("deltaDeg: ", deltaDeg);
                    }*/
                    //Identify correct button
                    //Color sensor and servo stuff
                    double bPos = button.getPosition();
                    telemetry.addData("R:", rgb.red());
                    telemetry.addData("G:", rgb.green());
                    telemetry.addData("B:", rgb.blue());
                    float[] hsv = {0,0,0};
                    Color.RGBToHSV((rgb.red() * 255) / 800, (rgb.green() * 255) / 800,
                            (rgb.blue() * 255) / 800, hsv);
                    telemetry.addData("H:", hsv[0]);
                    telemetry.addData("sPos:", button.getPosition());


                    if(bPos < .25 || bPos > .8) {
                        if(rgb.blue() > rgb.red()) {
                            //phase++;
                        }
                        else {
                            if (bPos < .25 && timer.time() > 4) {
                                timer.reset();
                                button.setPosition(MAX_POS);
                            }
                            else if (bPos > .8 && timer.time() > 4) {
                                timer.reset();
                                button.setPosition(MIN_POS);
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
            telemetry.update();
        }
    }

    /**
     * Shortcut to set the same power to all motors
     * @param power Motor power (-1 to 1)
     */
    public void setPower(double power)
    {
        power = ClampUnit(power);
        lfront.setPower(power);
        lback.setPower(power);
        rfront.setPower(power);
        rback.setPower(power);
    }

    /**
     * Shortcut to set two motors to the negative value of the other two motors (induce spin)
     * @param power Motor power (-1 to 1)
     */
    public void setSpinPower(double power)
    {
        power = ClampUnit(power);

        lfront.setPower(power);
        lback.setPower(power);
        //Right motors spin in reverse
        rfront.setPower(-power);
        rback.setPower(-power);
    }

    /**
     * Shortcut to assign two motors to turn slower than their opposites
     * @param base Motor power (-1 to 1)
     * @param ratio How much power to send to the drive side from the aux
     */
    public void setTurnPower(double base, double ratio)
    {
        double l = base * (1 + ratio);
        double r = base * (1 - ratio);

        l = ClampUnit(l);
        r = ClampUnit(r);

        lfront.setPower(l);
        lback.setPower(l);
        rfront.setPower(r);
        rback.setPower(r);
    }

    /**
     * Shortcut for retrieving pose data from VuforiaTrackable
     * @param name Name of beacon to get pose of
     * @param beacons List of all beacons
     * @return Beacon pose; null if not found
     */
    public OpenGLMatrix getPose(String name, VuforiaTrackables beacons)
    {
        OpenGLMatrix pose = null;
        for (VuforiaTrackable obj : beacons) {
            if (obj.getName().equals(name)) {
                pose = ((VuforiaTrackableDefaultListener) obj.getListener()).getPose();
                return pose;
            }
        }
        return pose;
    }

    /**
     * Shortcut for retrieving translation data from a VuforiaTrackable pose
     * @param name Name of beacon to get translation of
     * @param beacons List of all beacons
     * @return Beacon translation; null if not found
     */
    public VectorF getTranslation(String name, VuforiaTrackables beacons)
    {
        OpenGLMatrix tPose = getPose(name, beacons);
        if(tPose != null) { return tPose.getTranslation(); }
        return null;
    }

    /**
     * Calculates the degrees between camera forward vector and vector aimed at beacon
     * @param trans Translation of VuforiaTrackable
     * @return stuff
     */
    public double getDegrees(VectorF trans) {
        return Math.toDegrees(Math.atan2(trans.get(0), trans.get(2)));
    }

    /**
     * Returns the absolute difference between current heading and desired heading in degrees
     * @param deg Degree reading from vuforia
     * @return Difference in degrees between current heading and target heading
     */
    public double getDeltaDegrees(double deg) {
        return 180 - Math.abs(deg);
    }

    /**
     * Shortcut for clamping motor power between -1 and 1
     * @param value the value to clamp between -1 and 1
     * @return clamped value between -1 and 1
     */
    public double ClampUnit(double value)
    {
        return Clamp(value, -1, 1);
    }

    /**
     * Accepts a value and clamps it between a minimum and maximum value
     * @param value The value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Value clamped between min and max
     */
    public double Clamp(double value, double min, double max)
    {
        if(value < min )
            return min;
        else if(value > max)
            return max;
        return value;
    }
}
