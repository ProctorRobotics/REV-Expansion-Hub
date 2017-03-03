/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the drive and aux Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ProctorColorOp", group="TeleOp")
//@Disabled
public class ProctorColorTest extends OpMode{

    /* Declare OpMode members. */

    public DcMotorController left;
    public DcMotorController right;
    public DcMotor lfront;
    public DcMotor lback;
    public DcMotor rfront;
    public DcMotor rback;
    public ColorSensor color;

    public ServoController servos;
    public Servo button;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        right = hardwareMap.dcMotorController.get("aux");
        left = hardwareMap.dcMotorController.get("drive");

        lfront = hardwareMap.dcMotor.get("left");
        lback = hardwareMap.dcMotor.get("lback");
        rfront = hardwareMap.dcMotor.get("right");
        rback = hardwareMap.dcMotor.get("rback");

        servos = hardwareMap.servoController.get("servo");
        button = hardwareMap.servo.get("button");
        color = hardwareMap.colorSensor.get("rgb");

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

        button.setPosition(0.05);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {

        // Run wheels in tank mode

        double l = -gamepad1.left_stick_y;
        double r = -gamepad1.right_stick_y;

        //Precision movement
        if(gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0)
        {
            l *= 0.2;
            r *= 0.2;
        }

        lfront.setPower(l);
        lback.setPower(l);
        rfront.setPower(r);
        rback.setPower(r);

        if(gamepad1.left_bumper)
            button.setPosition(1);
        else if(gamepad1.right_bumper)
            button.setPosition(.05);

        telemetry.addData("R", color.red());
        telemetry.addData("G", color.green());
        telemetry.addData("B", color.blue());
        telemetry.addData("A", color.alpha());
        telemetry.addData("ARGB", color.argb());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
