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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 */
public class HardwareMechV2
{
    /* Public OpMode members. */

    public DcMotorEx  leftFront   = null;
    public DcMotorEx  rightFront  = null;
    public DcMotorEx leftBack  = null;
    public DcMotorEx  rightBack  = null;
    public DcMotorEx intake  = null;
    public DcMotorEx  robotLift     = null;
    public DcMotorEx  lift    = null;
   
    public Servo grabberFrontServo = null;
    public Servo grabberRearServo = null;
    public Servo pixelRotator = null;
    public Servo fourBarLiftRight   = null;
    public Servo fourBarLiftLeft = null;
    //public DistanceSensor distSensor =null;
    public Servo fourBarRotator = null;
    public Servo robotLiftAlignServo = null;
    public Servo droneLauncher = null;
    public Servo pixelPickerLinkage = null;
    public CRServo conveyor = null;

    public  DigitalChannel redLED;
    public DigitalChannel greenLED;
    public DigitalChannel breakBeamReciever;


    public DistanceSensor sensorDistance =null; //for REV color and dist sensor
    public ColorSensor sensorColor = null; // for REV color and dist sensor

    public static final double INIT_GRABBER     =  0.00;
    public static final double INIT_PIXROTATOR     =  0;
    public static final double INIT_FOURBARROTATOR    =  0.0;
    public static final double INIT_FOURBARLIFT   =  0.0;
    public static final double INIT_ROBOTLIFTALIGN = 0.0;
    public static final double INIT_DRONELAUNCHER = 0.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMechV2(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        leftFront  = hwMap.get(DcMotorEx.class, "leftFront");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        leftBack  = hwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");

        intake = hwMap.get(DcMotorEx.class, "intake");
        robotLift    = hwMap.get(DcMotorEx.class, "robotLift");
        lift   = hwMap.get(DcMotorEx.class, "lift");
        //intake    = hwMap.get(DcMotorEx.class, "intake");

        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        intake.setDirection(DcMotor.Direction.REVERSE);
        robotLift.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        //intake.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        intake.setPower(0);
        robotLift.setPower(0);
        lift.setPower(0);
        //intake.setPower(0);

       // leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       //leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // commented this out 2/6/2024 so lift position wont re-set at teleOpp init
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //turn encoders off to run faster, tip from DATA FORCE
        robotLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.


        pixelRotator = hwMap.get(Servo.class,"pixelRotator");

        grabberFrontServo = hwMap.get(Servo.class, "gripperFront");
        grabberRearServo=hwMap.get(Servo.class,"gripperBack");

        fourBarLiftRight= hwMap.get(Servo.class, "fourBarRight");
        fourBarLiftLeft= hwMap.get(Servo.class, "fourBarLeft");
        fourBarLiftRight.setDirection(Servo.Direction.REVERSE);

        fourBarRotator = hwMap.get(Servo.class, "fourBarRotator");
        fourBarRotator.setDirection(Servo.Direction.REVERSE);

        robotLiftAlignServo = hwMap.get(Servo.class, "robotLiftServo");

        droneLauncher = hwMap.get(Servo.class, "droneLauncher");
        conveyor = (CRServo) hwMap.get(CRServo.class, "conveyor");

        conveyor.setPower(0);

        pixelPickerLinkage = hwMap.get(Servo.class, "intakeExtender");
        pixelPickerLinkage.setDirection(Servo.Direction.REVERSE);



        //led
        redLED = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");

        //breakbeam
        //breakBeamReciever = hwMap.get(DigitalChannel.class, "breakBeamReciever");
        //breakBeamReciever.setMode(DigitalChannel.Mode.INPUT);
        //PixelSensor

// get a reference to the color sensor.
        //sensorColor = hwMap.get(ColorSensor.class, "sensorDistColor");

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hwMap.get(DistanceSensor.class, "sensorDistColor");

        greenLED.setState(true);
        redLED.setState(true);



        // distance sensor
        //distSensor = hwMap.get(DistanceSensor.class, "sensorDistColor");


    }
 }

