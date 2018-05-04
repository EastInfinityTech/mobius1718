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

//Below are needed for VuForia
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 *  This file illustrates the concept of driving a path based on time.
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

    public enum JewelColorType{unKnown,red,blue};

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive=null;
    private DcMotor rightDrive=null;
    private Servo leftFlipServo=null;
    private Servo rightFlipServo=null;
    protected Servo jewelServo = null;
    private ColorSensor colorSensor=null;

    private VuforiaTrackables relicTrackables=null;
    private VuforiaTrackable relicTemplate=null;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private void moveInches(int inchToMove, boolean moveForward){
        /* These are our observations for the relationship between time and distance travelled.
        *  If we assume this to be distance = Time * Factor + Constant
        *  Here Constant is the offset for the inital momentum
        *  Readings are as below on the test field:
        *  1000Ms we go between 14 1/4 to 14 1/2 inches (5 samples)
        *  2000Ms we go between 28 7/8 to  29 3/8 inches
        *  3000 Ms we go 44 3/8 inches
        *
        *  With Approximation we get Time = 200/3 * (distance+0.77)
        *  These are with Power Value 0.20
        */

        int timetoMove = (int)(200*(inchToMove+0.77)/3);  //Change Distance HERE (27 to any other number. Keep other factors same)

        double powerValueforSpeed = 0.20;
        if(!moveForward){
            powerValueforSpeed*=-1;
        }

        runtime.reset();
        while (runtime.milliseconds() < timetoMove) {
            rightDrive.setPower(powerValueforSpeed);
            leftDrive.setPower(powerValueforSpeed);
        }

        rightDrive.setPower(0);
        leftDrive.setPower(0);

        telemetry.addData("Time Moved:", timetoMove);
        telemetry.update();
    }

    protected void moveInchesForward(int inchToMove){
        moveInches(inchToMove,true);
    }

    protected void moveInchesBack(int inchToMove){
        moveInches(inchToMove,false);
    }

    protected void turnRight() {
        runtime.reset();
        while(runtime.milliseconds() < 1400) {
            leftDrive.setPower(.25);
            rightDrive.setPower(-.25);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        telemetry.addData("I Turned Right", 90);
        telemetry.update();
    }

    protected void turnLeft() {
        runtime.reset();
        while(runtime.milliseconds() < 1400) {
            leftDrive.setPower(-0.25);
            rightDrive.setPower(0.25);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        telemetry.addData("I Turned Left:", 90);
        telemetry.update();
    }

    protected JewelColorType findJewelColorType(){
        JewelColorType myJewelColor = JewelColorType.unKnown;
        int jewelRedColor;
        int jewelBlueColor;
        String strTest="";
        jewelRedColor = colorSensor.red();
        jewelBlueColor = colorSensor.blue();
         strTest = "Red Value:#" + String.valueOf(jewelRedColor) + "#, Blue Value:#"+ String.valueOf(jewelBlueColor)+"#";
        if (jewelRedColor>60)
        {
            myJewelColor = JewelColorType.red;
            strTest = "I call Red: " + strTest;
        }
        else if (jewelBlueColor>60)
        {
            myJewelColor = JewelColorType.blue;
            strTest = "I call Blue: " + strTest;
        }
        else
        {
            myJewelColor = JewelColorType.unKnown;
            strTest = "I DO NOT KNOW what to DO: " + strTest;
        }

        telemetry.addData("Color Value:", strTest);    //
        telemetry.update();

        return myJewelColor;
    }

    protected void initAutonRoutine()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftDrive=hardwareMap.get(DcMotor.class, "leftDriveMotor");
        rightDrive=hardwareMap.get(DcMotor.class, "rightDriveMotor");
        leftFlipServo=hardwareMap.get(Servo.class, "leftFlipServo");
        rightFlipServo=hardwareMap.get(Servo.class, "rightFlipServo");
        jewelServo=hardwareMap.get(Servo.class, "jewelKnocker");
        colorSensor=hardwareMap.get(ColorSensor.class,"ColorSensor");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();

        // Send telemetry message to signify robot waiting; Can I check in

         /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: This is our developer Key for Team Mobius
         */
        parameters.vuforiaLicenseKey = "ASlL9Qv/////AAAAmbelwcDPTkrNtXTexx98uR1/1VPslSYEnsQb+uBl7k7+cedVKMvImzLFOO1lKjo/a8kfniwV0pr/uwea4gCCmkqgcNL3bVdRaljdlo9Nkxb6hlm36887q1dunixliRyWAC2rhfB4CKqKJUWHlcgK9WYM9mQc0Jsi9GqVdDNEcBPIhNeVCYqr+iL0jCGpmjNXpwTiClnUMcVQiDivj99lrYzTK4KjHrQxD2XXBHqx234v/I0qial8E3xBdR8c2Ha+qS73pNyNFkIjgrfHPdHpxpEXkWL5FUJWXT6lWNqaMnEZv+GGn9Xxmvbo3OIOEpQ6WGEwEUXk1uxtgFBofDK8tTlGWKJgtm0Rn83M+8QZzCKC";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData("Status",  "Ready to run");    //
        telemetry.update();
    }

    protected void endAutonRoutine(){
        telemetry.addData("Status", "Time to stop at Home...");    //
        telemetry.update();

        sleep(1000);
        moveInchesBack(1);

        rightDrive.setPower(0);
        leftDrive.setPower(0);

        requestOpModeStop();
        sleep(1000);

    }

    protected RelicRecoveryVuMark processCipher(){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        relicTrackables.activate();

        //Try focusing for 2 seconds and find the cipher
        runtime.reset();
        while(runtime.milliseconds() < 4000) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                break;
            }
        }

        if (vuMark == RelicRecoveryVuMark.UNKNOWN){
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();
        return vuMark;
        }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    protected void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
