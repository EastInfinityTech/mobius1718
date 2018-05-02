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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * https://github.com/ftctechnh /ftc_app/wiki/Using-Two-Expansion-Hubs Read it later (connecting two hubs)
 */

@TeleOp(name="New_TankDrive", group="Iterative Opmode")
//@Disabled
public class Default_Short_TankDrive extends OpMode
{
    // Declare OpMode members.

    private DcMotor leftDriveMotor=null;
    private DcMotor rightDriveMotor=null;
    private DcMotor leftLiftMotor=null;
    private DcMotor rightLiftMotor=null;
    private DcMotor leftConveyorMotor=null;
    private DcMotor rightConveyorMotor=null;
    private DcMotor relicMotor=null;
    private Servo relicClampServo=null;
    private Servo relicArmServo=null;
    private Servo leftFlipServo=null;
    private Servo rightFlipServo=null;

    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        telemetry.addData("Status", "It Worked!");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

//        leftDriveMotor=hardwareMap.get(DcMotor.class, "leftDriveMotor");
//        rightDriveMotor=hardwareMap.get(DcMotor.class, "rightDriveMotor");
        leftLiftMotor=hardwareMap.get(DcMotor.class, "leftLiftMotor");
//        rightLiftMotor=hardwareMap.get(DcMotor.class, "rightLiftMotor");
        leftConveyorMotor=hardwareMap.get(DcMotor.class, "leftConveyorMotor");
        rightConveyorMotor=hardwareMap.get(DcMotor.class, "rightConveyorMotor");
        leftFlipServo=hardwareMap.get(Servo.class, "leftFlipServo");
        rightFlipServo=hardwareMap.get(Servo.class, "rightFlipServo");
//        relicMotor=hardwareMap.get(DcMotor.class, "relicMotor");
//       relicClampServo=hardwareMap.get(Servo.class, "relicClampServo");
        relicArmServo=hardwareMap.get(Servo.class, "relicArmServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

 //       leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
 //       rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
 //       rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftConveyorMotor.setDirection(DcMotor.Direction.REVERSE);
        rightConveyorMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFlipServo.setDirection(Servo.Direction.FORWARD);
        rightFlipServo.setDirection(Servo.Direction.REVERSE);
//        relicMotor.setDirection(DcMotor.Direction.FORWARD);
//        relicClampServo.setDirection(Servo.Direction.FORWARD);
        relicArmServo.setDirection(Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "It Worked!");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
    /
     */

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        float leftDrivePower=0;
        float rightDrivePower=0;
        float liftMotorPower=0;
        boolean ConveyorMotorON=false;
        float relicMotorPower=0;
        boolean relicClampServoON=false;
        boolean relicArmServoON=false;
        boolean flipServoON = false;


        // Tank Mode uses one stick to control each wheel.
        // - This requires no  math, but it is hard to drive forward slowly and keep straight.

        printButtonTraceValaues();

        leftDrivePower=gamepad1.left_stick_y/2;
        rightDrivePower=gamepad1.right_stick_y/2;

        liftMotorPower=rightDrivePower-leftDrivePower;//(gamepad1.right_trigger-gamepad1.left_trigger)/2;

        if (gamepad1.left_bumper){
            ConveyorMotorON=true;
        }
        if (gamepad1.right_bumper){
            ConveyorMotorON=false  ;
        }

/*        relicMotorPower=gamepad2.right_stick_y/2;
        if (gamepad2.a) {
            relicClampServoON = true;
        }
        if (gamepad2.b) {
            relicClampServoON = false;
        }

        if (gamepad2.left_bumper) {
            relicArmServoON = true;
        }
        if (gamepad2.left_bumper) {
            relicArmServoON  = false;
        }
*/

        if (gamepad1.x) {
            relicArmServoON = true;
        }
        if (gamepad1.y) {
            relicArmServoON  = false;
        }

        if (gamepad1.a) {
            flipServoON = true;
        }
        if (gamepad1.b) {
            flipServoON = false;
        }

//        leftDriveMotor.setPower(leftDrivePower);
//        rightDriveMotor.setPower(rightDrivePower);

//        rightLiftMotor.setPower(liftMotorPower);
//        leftLiftMotor.setPower(liftMotorPower);

        // Flip the plate only off or on (Need to see if control to control position is required later
        if(flipServoON){
            rightFlipServo.setPosition(0.5);
            leftFlipServo.setPosition(0.5);
        }
        else{
            rightFlipServo.setPosition(0);
            leftFlipServo.setPosition(0);
        }

        //Lets have Conveyor Motor run only on and off. No need to control the speed
        if(ConveyorMotorON){
            leftConveyorMotor.setPower(.4);
            rightConveyorMotor.setPower(.4);
        }
        else{
            leftConveyorMotor.setPower(0);
            rightConveyorMotor.setPower(0);
        }

        /*

        if(relicClampServoON){
            relicClampServo.setPosition(0.5);
        }
        else{
            relicClampServo.setPosition(0);`
        }
*/
        if(relicArmServoON){
            relicArmServo.setPosition(0.7);
        }
        else {
            relicArmServo.setPosition(0);
        }

//  private DcMotor relicMotor=null;= up and down
//  private Servo relicArmServo=null;= forward and backward

        // Show the elapsed game time and wheel power.
     //   telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftDrivePower, rightDrivePower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    // Set power=0 for all motors and servos
    @Override
    public void stop() {
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
    }

    private void printButtonTraceValaues()
    {

        String strBool;

        /*
         * These are float type
         * left_stick_x                 Range 0.0 to 1.0;
         * right_stick_x                Range 0.0 to 1.0;
         * left_stick_y                 Range 0.0 to 1.0;
         * right_stick_y                Range 0.0 to 1.0;
         * left_trigger                 Range 0.0 to 1.0;
         * right_trigger                Range 0.0 to 1.0;
         */

        /*
         * These are boolean type
         * gamepad1.left_bumper         true when Pressed;
         * gamepad1.right_bumper        true when Pressed;
         * gamepad1.a                   true when Pressed;
         * gamepad1.back                true when Pressed;
         * gamepad1.dpad_down           true when Pressed;
         * gamepad1.dpad_left           true when Pressed;
         * gamepad1.dpad_right          true when Pressed;
         * gamepad1.dpad_up             true when Pressed;
         * gamepad1.b                   true when Pressed;
         * guide                        true when Pressed;
         * left_stick_button            true when Pressed;
         * right_stick_button           true when Pressed;
         * start                        true when Pressed;
         * x                            true when Pressed;
         * y                            true when Pressed;
         */

        strBool="";

        telemetry.addData("Float Values", "left_stick_x:%.2f "+
                        "right_stick_x:%.2f left_stick_y:%.2f right_stick_y:%.2f left_trigger:%.2f "+
                        "right_trigger:%.2f ", gamepad1.left_stick_x,gamepad1.right_stick_x,
                gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger,
                gamepad1.right_trigger);

        if(gamepad1.left_bumper)
            strBool="left_bumper=ON ";
        else
            strBool="left_bumper=OFF ";
        if(gamepad1.right_bumper)
            strBool+="right_bumper=ON ";
        else
            strBool+="right_bumper=OFF ";
        if(gamepad1.a)
            strBool+="a=ON ";
        else
            strBool+="a=OFF ";
        if(gamepad1.back)
            strBool+="back=ON ";
        else
            strBool+="back=OFF ";
        if(gamepad1.dpad_down)
            strBool+="dpad_down=ON ";
        else
            strBool+="dpad_down=OFF ";
        if(gamepad1.dpad_left)
            strBool+="dpad_left=ON ";
        else
            strBool+="dpad_left=OFF ";
        if(gamepad1.dpad_right)
            strBool+="dpad_right=ON ";
        else
            strBool+="dpad_right=OFF ";
        if(gamepad1.dpad_up)
            strBool+="dpad_up=ON ";
        else
            strBool+="dpad_up=OFF ";
        if(gamepad1.b)
            strBool+="b=ON ";
        else
            strBool+="b=OFF ";
        if(gamepad1.guide)
            strBool+="guide=ON ";
        else
            strBool+="guide=OFF ";
        if(gamepad1.left_stick_button)
            strBool+="left_stick_button=ON ";
        else
            strBool+="left_stick_button=OFF ";
        if(gamepad1.right_stick_button)
            strBool+="right_stick_button=ON ";
        else
            strBool+="right_stick_button=OFF ";
        if(gamepad1.start)
            strBool+="start=ON ";
        else
            strBool+="start=ON ";
        if(gamepad1.x)
            strBool+="x=ON ";
        else
            strBool+="x=OFF ";
        if(gamepad1.y)
            strBool+="y=ON";
        else
            strBool+="y=OFF";

        telemetry.addData("Bool Values", strBool);

    }
}