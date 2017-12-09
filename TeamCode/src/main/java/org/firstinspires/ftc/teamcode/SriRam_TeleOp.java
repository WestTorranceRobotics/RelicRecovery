package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "One controller (v1.0)", group = "WTR")
//@Disabled

public class SriRam_TeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    public DcMotor leftMotor;
    public DcMotor leftMotorTwo;
    public DcMotor rightMotor;
    public DcMotor rightMotorTwo;
    //public DcMotor elevatorMotor;
    public DcMotor intakeMotorRight;
    public DcMotor intakeMotorLeft;
    public GyroSensor gyro;
    public ColorSensor color;
    private Servo rightramp;
    private Servo leftramp;
    private Servo rightDoor;
    private Servo leftDoor;
    private Servo jewelExcavator;
    public DcMotorEx lift;


    //public ServoController servoController;

    //300 servos
    /*
    public Servo dumpingServoLeft;
    public Servo dumpingSer
    voRight;

*/
    int liftState = 0;
    boolean flipUp = true;
    boolean doorOpen = false;
/*


    //180 servos
    public Servo leftDoorServo;
    public Servo rightDoorServo;*/


    @Override
    public void init() {
        //Motors
        leftMotor = hardwareMap.dcMotor.get("left1");
        leftMotorTwo = hardwareMap.dcMotor.get("left2");
        rightMotor = hardwareMap.dcMotor.get("right1");
        rightMotorTwo = hardwareMap.dcMotor.get("right2");
        intakeMotorLeft = hardwareMap.dcMotor.get("intake1");
        intakeMotorRight = hardwareMap.dcMotor.get("intake2");
        gyro = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        rightramp = hardwareMap.servo.get("servo1");
        leftramp = hardwareMap.servo.get("servo2");
        leftDoor = hardwareMap.servo.get("servo3");
        rightDoor = hardwareMap.servo.get("servo4");
        jewelExcavator = hardwareMap.servo.get("servo5");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("lift");

        //init lift
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lift has been init-ed
        leftramp.setPosition(0);
        rightramp.setPosition(1);
        leftDoor.setPosition(0.5);
        rightDoor.setPosition(0.5);
        jewelExcavator.setPosition(0);

        liftState = 0;

        flipUp = true;
        doorOpen = false;



        //servoController = hardwareMap.servoController.get("servo_controller");

        //300 servos
       /* dumpingServoLeft = hardwareMap.servo.get("dumping_servo_one");
        dumpingServoRight = hardwareMap.servo.get("dumping_servo_two");

        //180 servos
        leftDoorServo = hardwareMap.servo.get("leftdoorservo");
        rightDoorServo = hardwareMap.servo.get("rightdoorservo");*/


        //Modes
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotorTwo.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotorTwo.setDirection(DcMotor.Direction.FORWARD);

        intakeMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        lift.setPower(-0.1);
    }

    @Override
    public void start() {
        runtime.reset();
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setPower(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop() {

        //Drive Train
        if (Math.abs(gamepad1.right_stick_y) < .15) {
            leftMotor.setPower(0);
            leftMotorTwo.setPower(0);
        } else {
            leftMotor.setPower(gamepad1.right_stick_y);
            leftMotorTwo.setPower(gamepad1.right_stick_y);
        }
        if (Math.abs(gamepad1.left_stick_y) < .15) {
            rightMotor.setPower(0);
            rightMotorTwo.setPower(0);
        } else {
            rightMotor.setPower(gamepad1.left_stick_y);
            rightMotorTwo.setPower(gamepad1.left_stick_y);
        }


        //GamePad 2 code










        // Intake Code limited to 0.5 power

        /*if (Math.abs(gamepad1.left_stick_y) < .2) {
            intakeMotorLeft.setPower(0);
        } else {
            intakeMotorLeft.setPower(-gamepad1.left_stick_y * 0.5);
        }

        if (Math.abs(gamepad1.right_stick_y) < .2) {
            intakeMotorRight.setPower(0);
        } else {
            intakeMotorRight.setPower(-gamepad1.right_stick_y * 0.5);
        }
        */


        if(gamepad1.left_trigger > 0.18){
            intakeMotorLeft.setPower(-1);
        }
        else if (gamepad1.left_bumper){
            intakeMotorLeft.setPower(1);
        }
        else{
            intakeMotorLeft.setPower(0);
        }


        if(gamepad1.right_trigger > 0.18){
            intakeMotorRight.setPower(-1);
        }
        else if (gamepad1.right_bumper){
            intakeMotorRight.setPower(1);
        }
        else{
            intakeMotorRight.setPower(0);
        }


        //Make sure the stick is up
        if(jewelExcavator.getPosition() != 0) {
            jewelExcavator.setPosition(0);
        }
        // lift code
        switch (liftState){
            case(-1):
                lift.setTargetPosition(0);
                break;
            case(0):
                lift.setTargetPosition(-500);
                break;
            case(1):
                lift.setTargetPosition(-1100);
                break;
        }
        if(gamepad1.dpad_up){
            liftState = 1;
            lift.setPower(1);
        }
        if(gamepad1.dpad_down){
            liftState = -1;
            lift.setPower(0.6);
        }
        if(gamepad1.dpad_left || gamepad1.dpad_right){
            liftState = 0;
            lift.setPower(0.6);
        }


        //dump code
        //press x to reset; press y to dump
        if(gamepad1.x & !gamepad1.y && !flipUp){
            leftramp.setPosition(0);
            rightramp.setPosition(1);
            flipUp = true;
            leftDoor.setPosition(.5);
            rightDoor.setPosition(.5);
            doorOpen = false;
        }

        if(!gamepad1.x & gamepad1.y && flipUp){
            leftramp.setPosition(1);
            rightramp.setPosition(0);
            flipUp = false;
            leftDoor.setPosition(0);
            rightDoor.setPosition(1);
            doorOpen = true;
        }
        //door code
        if(gamepad1.a & !gamepad1.b && !doorOpen){
            leftDoor.setPosition(0);
            rightDoor.setPosition(1);
            doorOpen = true;
        }


        if(!gamepad1.a & gamepad1.b && doorOpen && flipUp){
            leftDoor.setPosition(.5);
            rightDoor.setPosition(.5);
            doorOpen = false;
        }
/*
        //telemetry
        telemetry.addData("leftPower", gamepad1.left_stick_y);
        telemetry.addData("rightPower", gamepad1.right_stick_y);
        //telemetry.addData("intakeIeftPower", gamepad1.left_stick_y);
        //telemetry.addData("intakeRightPower", gamepad1.right_stick_y);
        telemetry.addData("leftDoor", leftDoor.getPosition());
        telemetry.addData("rightDoor", rightDoor.getPosition());
        telemetry.addData("leftramp", leftramp.getPosition());
        telemetry.addData("rightramp", rightramp.getPosition());
        telemetry.addData("jewelExcavator", jewelExcavator.getPosition());*/
        telemetry.addData("R", color.red());
        telemetry.addData("G", color.green());
        telemetry.addData("B", color.blue());


        //Intake
        /*if (gamepad1.right_bumper){
            intakeMotorRight.setPower(1);
            intakeMotorLeft.setPower(1);
        }


        //Outtake
        if (gamepad1.left_bumper && !gamepad1.right_bumper){
            intakeMotorRight.setPower(-1);
            intakeMotorLeft.setPower(-1);
        }

        if (!gamepad1.right_bumper && !gamepad1.left_bumper){
            intakeMotorLeft.setPower(0);
            intakeMotorRight.setPower(0);
        }*/
        //open
       /* if (gamepad1.a) {
            setLeftDoorServo(-1);
            setRightDoorServo(1);
        }

        //close
        if (gamepad1.b) {
            setLeftDoorServo(0);
            setRightDoorServo(0);
        }

        //FLIPPP
        */
       /*
        if (gamepad1.y) {
            setLeftDumpServo(1);
            setRightDumpServo(1);
        } if (!gamepad1.y){
            setLeftDumpServo(0);
            setRightDumpServo(0);
        }
        */
        /*

        //upadoodles
        if (gamepad1.right_bumper) {
            elevatorMotor.setPower(1);
        }

        //downadoodles
        if (gamepad1.left_bumper) {
            elevatorMotor.setPower(-1);
            }*/
    }
}
