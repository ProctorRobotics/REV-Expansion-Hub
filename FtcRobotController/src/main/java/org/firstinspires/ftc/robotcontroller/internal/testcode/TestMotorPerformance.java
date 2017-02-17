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
package org.firstinspires.ftc.robotcontroller.internal.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This Opmode executes a basic Tank Drive Teleop for a PushBot
 * In addition, it allows manual transition from motor break to motor float by hitting the JS B button.
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Motor Performance Test", group="Test")  // @Autonomous(...) is the other common choice
@Disabled
public class TestMotorPerformance extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    double  leftPower ;
    double  rightPower ;
    boolean breaking = false;
    boolean lastBreakButton = false;
    boolean encoders = false;
    boolean lastEncoderButton = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Look to see if we should change breaking state
            if (gamepad1.b && !lastBreakButton)
            {
                breaking = !breaking;
                leftMotor.setZeroPowerBehavior(breaking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
                rightMotor.setZeroPowerBehavior(breaking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            }
            lastBreakButton = gamepad1.b;

            // Look to see if we should change encoder state
            if (gamepad1.x && !lastEncoderButton)
            {
                encoders = !encoders;
                if (encoders) {
                    leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                    leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
            lastEncoderButton = gamepad1.x;

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            telemetry.addData(">", "B for BRAKE/FLOAT. X for encoder toggle");
            telemetry.addData("Mode ", "%s (%.1f)", leftMotor.getMode(), runtime.time() );
            telemetry.addData("Idle ", "%s:%s", leftMotor.getZeroPowerBehavior(), rightMotor.getZeroPowerBehavior());
            telemetry.addData("Power", "%5.2f:%5.2f", leftPower, rightPower);
            telemetry.update();

            sleep(50);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
