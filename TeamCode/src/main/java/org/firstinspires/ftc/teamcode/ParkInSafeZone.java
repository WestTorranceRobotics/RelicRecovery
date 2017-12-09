package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by WTR on 11/8/2017.
 */
@Autonomous(name="ParkInSafeZone", group="WTR")
@Disabled
public class ParkInSafeZone extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();

    private enum state {
        STATE_MOVE_FORWARD,
        STATE_TURN_NINETY,
        STATE_MOVE_MORE,

    }

    private state currentState;

    //Motors
    public DcMotor leftMotorTwo;
    public DcMotor leftMotor;
    public DcMotor rightMotorTwo;
    public DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_motor_front");
        leftMotorTwo = hardwareMap.dcMotor.get("left_motor_back");
        rightMotor = hardwareMap.dcMotor.get("right_motor_front");
        rightMotorTwo = hardwareMap.dcMotor.get("right_motor_back");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotorTwo.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotorTwo.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void init_loop() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    @Override
    public void start(){
        runtime.reset();
        newState(state.STATE_MOVE_FORWARD);
    }

    @Override
    public void loop() {
        switch (currentState) {
            case STATE_MOVE_FORWARD:
                setPos(36,1);
                newState(state.STATE_TURN_NINETY);
                break;

            case STATE_TURN_NINETY:
                if (stateTime.time() <=1.5) {
                    leftMotor.setPower(1);
                    rightMotor.setPower(-1);
                } else {
                    newState(state.STATE_MOVE_MORE);
                }
                break;

            case STATE_MOVE_MORE:
                setPos(12,1);
                break;
        }

    }


    public void stop(){
    }

    private void newState(state newState) {
        stateTime.reset();
        currentState = newState;
    }

    public void setPos(double inches, double goes){
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = (int) (inches * (89.1268));
        //adjust inch to tick conversion to new robot

        int currentLeft = leftMotor.getCurrentPosition();
        int currentRight = rightMotor.getCurrentPosition();

        leftMotor.setTargetPosition(ticks + currentLeft);
        rightMotor.setTargetPosition(ticks + currentRight);
        leftMotor.setPower(goes);
        rightMotor.setPower(goes);
    }

    public double currentPos = 0;
}
