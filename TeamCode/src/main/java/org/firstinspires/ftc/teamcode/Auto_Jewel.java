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

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ColorSensor colorSensor = null;
    private Servo jewelServo = null;
    private Servo armServo = null;
    private int jewelColor;
    protected boolean areWeRed = false;
    protected boolean areWeFront = false;

    public void runOpModeMain() {

        double powerValueforSpeed;
        int timeFor90DegreeTurnMs = 1200;
        int timetoMoveForJewel;
        int timetoMoveForwardFirstFront;
        int timetoMoveForwardFirstValue;
        int timetoMoveAfterTurnFront;
        int timetoMoveForwardFirstRear;
        int timetoAdjustRear;
        int turnToKnockoffJewel=400;
        int knockOffSeenJewelFactor=1;
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

        /* These are our observations for the relationship between time and distance travelled.
        *  If we assume this to be distance = Time * Factor + Constant
        *  Here Constant is the offset for the inital momentum
        *  Readings are as below on the test field:
        *  1000Ms we go between 14 1/4 to 14 1/2 inches (5 samples)
        *  2000Ms we go between 28 7/8 to  29 3/8 inches
        *  3000 Ms we go 44 3/8 inches
        *
        *  With Approximation we get Time = 200/3 * (distance+0.77)
        */

        timetoMoveForwardFirstFront = (int)(200*(27+0.77)/3);  //Change Distance HERE (27 to any other number. Keep other factors same
        timetoMoveForwardFirstRear = (int)(200*(18+0.77)/3);  //Change Distance HERE (10 to any other number. Keep other factors same
        timetoMoveAfterTurnFront= (int)(200*(5+0.77)/3);  //Change Distance HERE (10 to any other number. Keep other factors same
        timetoAdjustRear= (int)(200*(1+0.77)/3);  //Change Distance HERE (1 to any other number. Keep other factors same
        timetoMoveForJewel= (int)(200*(3+0.77)/3);  //Change Distance HERE (3 to any other number. Keep other factors same

        powerValueforSpeed=0.20;
        if (areWeFront) {
            timetoMoveForwardFirstValue = timetoMoveForwardFirstFront;
            telemetry.addData("Robot Placement Type:", "I am Front");    //
            telemetry.update();
        }
        else {
            timetoMoveForwardFirstValue = timetoMoveForwardFirstRear;
            telemetry.addData("Robot Placement Type:", "I am Back");    //
            telemetry.update();
        }
        if (areWeRed) {
            powerValueforSpeed = powerValueforSpeed*-1; //Reverse the Power Value to take care of Turn Type for Red
            telemetry.addData("Team Color:", "Red");    //
            telemetry.update();
        }
        else {
            telemetry.addData("Team Color:", "Blue");    //
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        /****************************************************************************************
        //This code is disabled for 4-Nov as we are not going to use Jewel Knocker)
        runtime.reset();
        while (runtime.milliseconds()    < timetoMoveForJewel) { //Move backward
             rightDrive.setPower(-Math.abs(powerValueforSpeed));
             leftDrive.setPower(-Math.abs(powerValueforSpeed));
        }
        telemetry.addData("Status", "Went Back now Put Servo Down.");    //
        telemetry.update();

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        jewelServo.setPosition(-.60);
        jewelColor = colorSensor.red();

        knockOffSeenJewelFactor=1;
        runtime.reset();
        if (jewelColor >= 100) { //Red
            telemetry.addData("Status", "I see Red Color Jewel.");    //
            telemetry.update();
            if (areWeRed) {
                knockOffSeenJewelFactor = -1;
            }
        }
         else{ //Blue
            telemetry.addData("Status", "I see Blue Color Jewel");    //
            telemetry.update();
            if (!areWeRed) {
                knockOffSeenJewelFactor=-1;
            }
         }

         runtime.reset();
         while (runtime.milliseconds() < turnToKnockoffJewel) { //Turn in direction of oposite color jewel
             rightDrive.setPower(knockOffSeenJewelFactor*.3);
             leftDrive.setPower(knockOffSeenJewelFactor*-.3);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        jewelServo.setPosition(.60);

        runtime.reset();
        while (runtime.milliseconds() < turnToKnockoffJewel) { //Turn back to Original direction
            rightDrive.setPower(knockOffSeenJewelFactor*-.3);
            leftDrive.setPower(knockOffSeenJewelFactor*.3);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Come backto original position
        runtime.reset();
        while (runtime.milliseconds()    < timetoMoveForJewel) { //Move backward
            rightDrive.setPower(Math.abs(powerValueforSpeed));
            leftDrive.setPower(Math.abs(powerValueforSpeed));
        }

        // Go Straight Ahead and out of Balancing Platform
        runtime.reset();
        while(runtime.milliseconds() < timetoMoveAfterTurnFront) { ///to be changed with new variable
            rightDrive.setPower(powerValueforSpeed);
            leftDrive.setPower(powerValueforSpeed);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn to Left or Right
        runtime.reset();
        while(runtime.milliseconds() < timeFor90DegreeTurnMs) {
            leftDrive.setPower(-powerValueforSpeed);
            rightDrive.setPower(powerValueforSpeed);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

         ***************************************************************************************/

//Current program starts working here. Code before this will not execute.
        // Go Straight Ahead
        telemetry.addData("Time", timetoMoveForwardFirstValue);    //
        telemetry.update();
        //Hold Glyph
        armServo.setPosition(.5);

        runtime.reset();
        while(runtime.milliseconds() < timetoMoveForwardFirstValue) {
            rightDrive.setPower(Math.abs(powerValueforSpeed*1.2));
            leftDrive.setPower(Math.abs(powerValueforSpeed*1.2));
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        if (areWeFront) {//We need to Turn and go to the safe Zone
            runtime.reset();
            while(runtime.milliseconds() < timeFor90DegreeTurnMs) {
                leftDrive.setPower(-powerValueforSpeed);
                rightDrive.setPower(powerValueforSpeed);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            runtime.reset();
            while(runtime.milliseconds() < timetoMoveAfterTurnFront) { //Go Straight
                rightDrive.setPower(Math.abs(powerValueforSpeed));
                leftDrive.setPower(Math.abs(powerValueforSpeed));
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        else{//Turn other way to set position
            runtime.reset();
            while(runtime.milliseconds() < timeFor90DegreeTurnMs) {
                leftDrive.setPower(powerValueforSpeed);
                rightDrive.setPower(-powerValueforSpeed);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            runtime.reset();
            while(runtime.milliseconds() < timetoAdjustRear) {
                rightDrive.setPower(Math.abs(powerValueforSpeed));
                leftDrive.setPower(Math.abs(powerValueforSpeed));
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            //Turn again to put Glyph
            runtime.reset();
            while(runtime.milliseconds() < timeFor90DegreeTurnMs) {
                leftDrive.setPower(-powerValueforSpeed);
                rightDrive.setPower(powerValueforSpeed);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

        }

        telemetry.addData("Status", "Time to Open the Arms...");    //
        telemetry.update();

        //Drop Glyph
        armServo.setPosition(-.5);
        armServo.setPosition(-.5);
        armServo.setPosition(-.5);

        rightDrive.setPower(0);
        leftDrive.setPower(0);
        requestOpModeStop();
        sleep(1000);
    }
}
