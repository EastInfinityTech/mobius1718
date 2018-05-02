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
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    protected ColorSensor colorSensor = null;
    protected Servo jewelServo = null;
    protected Servo armServo = null;
    private VuforiaTrackables relicTrackables=null;
    private VuforiaTrackable relicTemplate=null;

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
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armServo = hardwareMap.get(Servo.class, "armServo");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");
        colorSensor = hardwareMap.get(ColorSensor.class,"ColorSensor");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        jewelServo.setDirection(Servo.Direction.FORWARD);
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
        telemetry.addData("Status", "Time to Open the Arms...");    //
        telemetry.update();

        //Drop Glyph
        armServo.setPosition(-.5);
        armServo.setPosition(-.5);
        armServo.setPosition(-.5);
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

        //Try focussing for 2 seconds and find the cipher
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
}
