package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by WTR on 11/10/2017.
 */

@Autonomous(name = "ParkRight", group = "WTR")
//@Disabled
public class ParkRight extends OpMode {

    //States
    private enum State {
        STATE_INIT,
        STATE_PARK,
        STATE_STOP
    }

    final double TICKS_PER_INCH = 89.1268;

    //Hardware
    public DcMotor rLeftMotor;
    public DcMotor rRightMotor;

    private int rLeftEncoderTarget;
    private int rRightEncoderTarget;

    //time variables
    public ElapsedTime rRuntime = new ElapsedTime();
    private ElapsedTime rStateTime = new ElapsedTime();


    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }

    //functions
    private void newState(State newState) {
        rStateTime.reset();
        // rCurrentState = newState;
    }

    void setEncoderTarget(int leftEncoder, int rightEncoder) {
        rLeftMotor.setTargetPosition(rLeftEncoderTarget = leftEncoder);
        rRightMotor.setTargetPosition(rRightEncoderTarget = rightEncoder);
    }

    void addEncoderTarget(int leftEncoder, int rightEncoder) {
        rLeftMotor.setTargetPosition(rLeftEncoderTarget += leftEncoder);
        rRightMotor.setTargetPosition(rLeftEncoderTarget += rightEncoder);
    }

    void setDrivePower(double leftPower, double rightPower) {
        rLeftMotor.setPower(Range.clip(leftPower, -1, 1));
        rRightMotor.setPower(Range.clip(rightPower, -1, 1));
    }

    void setDriveSpeed(double leftSpeed, double rightSpeed) {
        setDrivePower(leftSpeed, rightSpeed);
    }

    void synchEncoders() {
        rLeftEncoderTarget = rLeftMotor.getCurrentPosition();
        rRightEncoderTarget = rRightMotor.getCurrentPosition();
    }

    int getrLeftEncoderPosition() {
        return rLeftMotor.getCurrentPosition();
    }

    int getrRightEncoderPosition() {
        return rRightMotor.getCurrentPosition();
    }

    boolean moveComplete() {
        return ((Math.abs(getrLeftEncoderPosition() - rLeftEncoderTarget) < 10) &&
                (Math.abs(getrRightEncoderPosition() - rRightEncoderTarget) < 10));
    }

    boolean encodersAtZero() {
        return ((Math.abs(getrLeftEncoderPosition()) < 5) &&
                Math.abs(getrRightEncoderPosition()) < 5);
    }
}



