package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.ColorDrawable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "RelicRecoveryTeleOp", group = "WTR")
//@Disabled

public class RelicRecoveryTeleOp extends OpMode {

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


    //public ServoController servoController;

    //300 servos
    /*public Servo dumpingServoLeft;
    public Servo dumpingSer
    voRight;

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
        //elevatorMotor = hardwareMap.dcMotor.get("elevator_motor");
        intakeMotorRight = hardwareMap.dcMotor.get("intake2");
        intakeMotorLeft = hardwareMap.dcMotor.get("intake1");
        gyro = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        leftramp = hardwareMap.servo.get("servo1");
        rightramp = hardwareMap.servo.get("servo2");
        leftDoor = hardwareMap.servo.get("servo3");
        rightDoor = hardwareMap.servo.get("servo4");
        jewelExcavator = hardwareMap.servo.get("servo5");


        //servoController = hardwareMap.servoController.get("servo_controller");

        //300 servos
       /* dumpingServoLeft = hardwareMap.servo.get("dumping_servo_one");
        dumpingServoRight = hardwareMap.servo.get("dumping_servo_two");

        //180 servos
        leftDoorServo = hardwareMap.servo.get("leftdoorservo");
        rightDoorServo = hardwareMap.servo.get("rightdoorservo");*/


        //Modes
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotorTwo.setDirection(DcMotor.Direction.FORWARD);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotorTwo.setDirection(DcMotor.Direction.REVERSE);

        intakeMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorRight.setDirection(DcMotor.Direction.REVERSE);
        //elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    boolean liftUp = false;
    boolean gamepadAWas = false;
    boolean gamepadAIs;

    boolean flipUp = false;
    boolean gamepadBWas = false;
    boolean gamepadBIs;

    @Override
    public void loop() {
        boolean liftUp = false;
        boolean gamepadAWas = false;
        boolean gamepadAIs;

        boolean flipUp = false;
        boolean gamepadBWas = false;
        boolean gamepadBIs;


        //Drive Train
        if (Math.abs(gamepad1.left_stick_y) < .15) {
            leftMotor.setPower(0);
            leftMotorTwo.setPower(0);
        } else {
            leftMotor.setPower(gamepad1.left_stick_y);
            leftMotorTwo.setPower(gamepad1.left_stick_y);
        }
        if (Math.abs(gamepad1.right_stick_y) < .15) {
            rightMotor.setPower(0);
            rightMotorTwo.setPower(0);
        } else {
            rightMotor.setPower(gamepad1.right_stick_y);
            rightMotorTwo.setPower(gamepad1.right_stick_y);
        }


        //GamePad 2 code


        // Intake Code limited to 0.75 power
        if (Math.abs(gamepad2.left_stick_y) < .1) {
            intakeMotorLeft.setPower(0);
        } else {
            intakeMotorLeft.setPower(gamepad2.left_stick_y * 0.5);
        }


        if (Math.abs(gamepad2.right_stick_y) < .1) {
            intakeMotorRight.setPower(0);
        } else {
            intakeMotorRight.setPower(gamepad2.right_stick_y * 0.5);
        }


        // code for dumping

        if (gamepad2.x & !gamepad2.y && !flipUp) {
            leftramp.setPosition(1);
            rightramp.setPosition(0);
            rightDoor.setPosition(0.5);
            leftDoor.setPosition(0.5);
            flipUp = true;
        }
        if (!gamepad2.x & gamepad2.y && flipUp) {
            leftramp.setPosition(0);
            rightramp.setPosition(1);
            rightDoor.setPosition(1);
            leftDoor.setPosition(0);
            flipUp = false;
        }
        telemetry.addData("leftPower", gamepad1.left_stick_y);
        telemetry.addData("leftpower", gamepad1.left_stick_y);

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
       /* if (gamepad2.a) {
            setLeftDoorServo(-1);
            setRightDoorServo(1);
        }

        //close
        if (gamepad2.b) {
            setLeftDoorServo(0);
            setRightDoorServo(0);
        }

        //FLIPPP
        */
       /*
        if (gamepad2.y) {
            setLeftDumpServo(1);
            setRightDumpServo(1);
        } if (!gamepad2.y){
            setLeftDumpServo(0);
            setRightDumpServo(0);
        }
        */
        /*

        //upadoodles
        if (gamepad2.right_bumper) {
            elevatorMotor.setPower(1);
        }

        //downadoodles
        if (gamepad2.left_bumper) {
            elevatorMotor.setPower(-1);
            }*/
    }
}