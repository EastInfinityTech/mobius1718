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
    private int jewelColor;
    private int timeFor90DegreeTurnMs = 1325;
    private int timeForTENInches = 300;
    private int timetoMoveBeforeFirstTurn=0;
    private int timetoMoveAfterFirstTurn=0;
    private int timetoMoveAfterSecondTurn=0;
    protected boolean areWeRed = false;
    protected boolean areWeFront = false;

    private double powerValueforSpeedPlus=0;

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

        telemetry.addData("Status",  "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        powerValueforSpeedPlus=0;
        timetoMoveBeforeFirstTurn= timeForTENInches*10/10;
        timetoMoveAfterFirstTurn = timeForTENInches*21/10;
        timetoMoveAfterSecondTurn= timeForTENInches*15/10;


        waitForStart();

        runtime.reset();
        while (runtime.milliseconds()    < 500) { //Move backward
            leftDrive.setPower(-powerValueforSpeedPlus);
            rightDrive.setPower(-powerValueforSpeedPlus);
        }
        telemetry.addData("Status", "Went Back now Put Servo Down.");    //
        telemetry.update();

        leftDrive.setPower(0);
        rightDrive.setPower(0);

     //   jewelServo.setPosition(-.60);
        jewelColor = colorSensor.red();

        runtime.reset();
        if (jewelColor >= 100) { //Red
            telemetry.addData("Status", "I see Red Color Jewel.");    //
            telemetry.update();
            while (runtime.milliseconds() < 300) { //Turn in direction of red jewel
                rightDrive.setPower(powerValueforSpeedPlus);
                leftDrive.setPower(-powerValueforSpeedPlus);
            }
        }
        else{ //Blue
            telemetry.addData("Status", "I see Blue Color Jewel");    //
            telemetry.update();
            while(runtime.milliseconds() < 300) { //Turn in direction of red jewel
                rightDrive.setPower(-powerValueforSpeedPlus);
                leftDrive.setPower(powerValueforSpeedPlus);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

//        jewelServo.setPosition(.60);

        if (jewelColor >= 100) { //Red
            while (runtime.milliseconds() < 300) { //Turn back to original position
                rightDrive.setPower(-powerValueforSpeedPlus);
                leftDrive.setPower(powerValueforSpeedPlus);
            }
        }
        else{ //Blue
            while(runtime.milliseconds() < 300) { //Turn back to original position
                rightDrive.setPower(powerValueforSpeedPlus);
                leftDrive.setPower(-powerValueforSpeedPlus);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);




//Current program starts working here. Code before this will not execute.
        powerValueforSpeedPlus=0.20;
        // Go Straight Ahead and out of Balancing Platform
        runtime.reset();
        while(runtime.milliseconds() < timetoMoveBeforeFirstTurn) { //Straight ahead for 4 second
            rightDrive.setPower(powerValueforSpeedPlus);
            leftDrive.setPower(powerValueforSpeedPlus);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn to Left or Right
        runtime.reset();
        while(runtime.milliseconds() < timeFor90DegreeTurnMs) { //Straight ahead for 4 second
            if (areWeRed) {
                rightDrive.setPower(-powerValueforSpeedPlus);
                leftDrive.setPower(powerValueforSpeedPlus);
            }
            else {
                leftDrive.setPower(-powerValueforSpeedPlus);
                rightDrive.setPower(powerValueforSpeedPlus);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Go Straight Ahead
        runtime.reset();
        while(runtime.milliseconds() < 1000) { //Straight ahead for 4 seconds
            rightDrive.setPower(powerValueforSpeedPlus);
            leftDrive.setPower(powerValueforSpeedPlus);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
/*
        runtime.reset();
        if (areWeFront) {
            if (areWeRed) {
                while(runtime.milliseconds() < timeFor90DegreeTurnMs) { //Straight ahead for 4 second
                    rightDrive.setPower(-powerValueforSpeedPlus);
                    leftDrive.setPower(powerValueforSpeedPlus);
                }
            }
            else {
                while(runtime.milliseconds() < timeFor90DegreeTurnMs) { //Straight ahead for 4 second
                    leftDrive.setPower(-powerValueforSpeedPlus);
                    rightDrive.setPower(powerValueforSpeedPlus);
                }
            }
        }
*/
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        requestOpModeStop();
        sleep(1000);
    }
}
