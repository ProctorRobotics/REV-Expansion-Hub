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

@TeleOp(name="ProctorTeleOp", group="TeleOp")
//@Disabled
public class ProctorTeleOp extends OpMode{

    /* Declare OpMode members. */

    public DcMotorController drive;
    public DcMotorController aux;
    public DcMotor left;
    public DcMotor right;
    public DcMotor hDrive;

    public ServoController servos;
    public Servo button;

    public boolean roller = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        aux = hardwareMap.dcMotorController.get("aux");
        drive = hardwareMap.dcMotorController.get("drive");

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        hDrive = hardwareMap.dcMotor.get("hdrive");

        servos = hardwareMap.servoController.get("servo");
        button = hardwareMap.servo.get("button");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        hDrive.setDirection(DcMotor.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set all motors to zero power
        left.setPower(0);
        right.setPower(0);
        hDrive.setPower(0);

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
        double h = -gamepad1.left_trigger + gamepad1.right_trigger;

        //Precision movement
        if(gamepad1.left_bumper || gamepad1.right_bumper)
        {
            l *= 0.2;
            r *= 0.2;
            h *= 0.4;
        }

        left.setPower(l);
        right.setPower(r);


        if(gamepad1.left_bumper)
            button.setPosition(1);
        else if(gamepad1.right_bumper)
            button.setPosition(.05);

        hDrive.setPower(h);

        telemetry.addData("left stick", -gamepad1.left_stick_y);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
