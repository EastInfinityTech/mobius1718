/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Disabled
public abstract class Auto_Jewel extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftDrive = null;
    protected DcMotor rightDrive = null;
    protected ColorSensor colorSensor = null;
    protected Servo jewelServo = null;
    protected Servo armServo = null;
    protected int jewelColor;
    protected boolean areWeRed = false;
    protected boolean areWeFront = false;

    public void runOpModeMain() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "In the Common Routine");    //
        telemetry.update();
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armServo = hardwareMap.get(Servo.class, "armServo");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");
        colorSensor = hardwareMap.get(ColorSensor.class,"ColorSensor");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        jewelServo.setDirection(Servo.Direction.FORWARD);
        // Send telemetry message to signify robot waiting; Can I check in

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        while (runtime.seconds() < 1) { //Move backward for 1 second
            leftDrive.setPower(-.20);
            rightDrive.setPower(-.20);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        jewelServo.setPosition(-.25);
        jewelColor = colorSensor.red();

        runtime.reset();
        if (jewelColor >= 100) { //Red
            while (runtime.seconds() < 2) { //Turn in direction of red jewel
                rightDrive.setPower(.20);
                leftDrive.setPower(-.20);
            }
        }
        else{ //Blue
            while(runtime.seconds() < 2) { //Turn in direction of red jewel
                rightDrive.setPower(-.20);
                leftDrive.setPower(.20);
            }
        }

        jewelServo.setPosition(.25);

        if (jewelColor >= 100) { //Red
            while (runtime.seconds() < 2) { //Turn back to original position
                rightDrive.setPower(-.20);
                leftDrive.setPower(.20);
            }
        }
        else{ //Blue
            while(runtime.seconds() < 2) { //Turn back to original position
                rightDrive.setPower(.20);
                leftDrive.setPower(-.20);
            }
        }

        // Go Straight Ahead and out of Balancing Platform
        runtime.reset();
        while(runtime.seconds() < 4) { //Straight ahead for 4 second
            rightDrive.setPower(.20);
            leftDrive.setPower(.20);
        }
        // Turn to Left or Right
        runtime.reset();
        while(runtime.seconds() < 2) { //Straight ahead for 4 second
            if (areWeRed) {
                rightDrive.setPower(.20);
                leftDrive.setPower(-.20);
            }
            else {
                leftDrive.setPower(.20);
                rightDrive.setPower(-.20);
            }
        }

        // Go Straight Ahead
        runtime.reset();
        while(runtime.seconds() < 4) { //Straight ahead for 4 seconds
            rightDrive.setPower(.20);
            leftDrive.setPower(.20);
        }

        runtime.reset();
        if (areWeFront) {
            while (runtime.seconds() < 2) { //Turn again for Front
                rightDrive.setPower(.20);
                leftDrive.setPower(-.20);
            }
            runtime.reset();
            while(runtime.seconds() < 4) { //Straight ahead for 4 seconds
                rightDrive.setPower(.20);
                leftDrive.setPower(.20);
            }
        }


        rightDrive.setPower(0);
        leftDrive.setPower(0);
        requestOpModeStop();
        sleep(1000);
    }
}
