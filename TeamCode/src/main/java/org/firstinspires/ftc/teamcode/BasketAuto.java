package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")

public class BasketAuto extends RobotLinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //the left front drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right front drivetrain motor
    public DcMotor  rightBackDrive  = null; //the right back drivetrain motor
    public DcMotor  leftBackDrive  = null; //the left back drivetrain motor
    public DcMotor  armMotor    = null; //the arm motor
    public DcMotor  VSlide   = null; //the left arm motor
    public CRServo intake = null;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {
        declareHardwareProperties();
        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive"); //the left front drivetrain motor
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //the right front drivetrain motor
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //the left drivetrain motor
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive"); //the left drivetrain motor
        armMotor  = hardwareMap.get(DcMotor.class, "arm_motor"); //the arm motor
        VSlide = hardwareMap.get(DcMotor.class, "vslide");
        intake = hardwareMap.get(CRServo.class, "intake");
         waitForStart();

         while(opModeIsActive()){
             encoderDrive(0.5, 10, MOVEMENT_DIRECTION.FORWARD);
             encoderTurn(0.5, 180, TURN_DIRECTION.TURN_LEFT);
             encoderSlideUp(0.5, 3, MOVEMENT_DIRECTION.FORWARD);
         }
    }
}

