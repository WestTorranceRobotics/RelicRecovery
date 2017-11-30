package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by WTR on 11/10/2017.
 */

@Autonomous(name = "LinearGetJewel", group = "WTR")
//@Disabled
public class LinearGetJewel extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftMotor;
    public DcMotor leftMotorTwo;
    public DcMotor rightMotor;
    public DcMotor rightMotorTwo;
    public ModernRoboticsI2cColorSensor colorsensor;


    @Override
    public void runOpMode(){
        leftMotor = hardwareMap.dcMotor.get("left_motor_front");
        leftMotorTwo = hardwareMap.dcMotor.get("left_motor_back");
        rightMotor = hardwareMap.dcMotor.get("right_motor_front");
        rightMotorTwo = hardwareMap.dcMotor.get("right_motor_back");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotorTwo.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotorTwo.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

       //turnLeft(-1);
       //sleep(750);
       driveForward(-1);
       sleep(2000);


    }

    public void driveForward(double power){
        leftMotor.setPower(power);
        leftMotorTwo.setPower(power);
        rightMotor.setPower(power);
        rightMotorTwo.setPower(power);
    }

    public void driveForwardTime(double power, long time) throws InterruptedException{
        driveForward(power);
        Thread.sleep(time);
    }

    public void stopDriving(){
        driveForward(0);
    }

    public void turnLeft(double power){
        leftMotor.setPower(power);
        leftMotorTwo.setPower(power);
        rightMotor.setPower(-power);
        rightMotor.setPower(-power);
    }

    public void turnLeftTime(double power, long time) throws InterruptedException{
        turnLeft(power);
        Thread.sleep(time);
    }

    public void turnRight(double power){
       turnLeft(-power);
    }

    public void turnRightTime(double power,long time) throws InterruptedException{
        turnRight(power);
        Thread.sleep(time);
    }
    public boolean colorIsBlue(){
        return (colorsensor.blue() > 20) && (colorsensor.red() < 15);
    }
}
