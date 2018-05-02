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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**Random comment
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * when we wanted to eat our potatoes, mr. ladida stopped us.
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * Haha you found this.  Good for you /_(*-*)_/
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Default_TankDrive", group="Iterative Opmode")
//@Disabled
public class Default_TankDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo armServo = null;
    private DcMotor elevatorMotor = null;
    private Servo jewelServo = null;

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = 0;     // Maximum REV power applied to motor
    double  power   = 0;
    boolean rampUp  = true;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armServo = hardwareMap.get(Servo.class, "armServo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        jewelServo.setDirection(Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        jewelServo.setPosition(-.6);
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        boolean elevatorUp;
        boolean elevatorDown;
        boolean armOpen;
        boolean armClose;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward :0 slowly and keep straight.
        leftPower  = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        elevatorUp=false;
        elevatorDown=false;
        if (gamepad1.right_trigger>0) {
            elevatorUp = true;
            //elevatorUp = gamepad1.dpad_up;
        }
        else if (gamepad1.left_trigger>0) {
            elevatorDown =true;
             //elevatorDown = gamepad1.dpad_down;
        }

        armOpen = gamepad1.left_bumper;
        armClose =gamepad1.right_bumper;

        //armOpen = gamepad1.x;
        //armClose = gamepad1.a;


        if (Math.abs(leftPower) + Math.abs(rightPower) >= .01) {
            // Keep stepping up until we hit the max value.
            power += INCREMENT ;
            if (power >= 1 ) {
                power = 1;
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            power -= INCREMENT ;
            if (power <= .1 ) {
                power = 0;
            }
        }

        leftDrive.setPower(leftPower*power);
        rightDrive.setPower(rightPower*power);

        //Elevator Mechanism
        if(elevatorUp){
            elevatorMotor.setPower(1);
        }
        else if(elevatorDown){
            elevatorMotor.setPower(-1);
        }
        else {
            elevatorMotor.setPower(0);
        }

        //Glyph Mechanism
        if(armOpen){
            armServo.setPosition(.5);
        }
        else if(armClose){
            armServo.setPosition(-.5);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower*power, rightPower*power);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}