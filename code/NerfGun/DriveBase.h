class DriveBase
    : public CoopTask
    , public RobotDrive
{
private:

public:
    void Stop(void)
    {
        StopMotor();
    }   //Stop

    DriveBase(
        SpeedController *leftFrontMotor,
        SpeedController *leftRearMotor,
        SpeedController *rightFrontMotor,
        SpeedController *rightRearMotor
        ): RobotDrive(leftFrontMotor, leftRearMotor,
                      rightFrontMotor, rightRearMotor)
    {
        //
        // Initialize RobotDrive.
        //
        SetInvertedMotor(RobotDrive::kFrontLeftMotor,
                         MOTOR_LEFT_FRONT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearLeftMotor,
                         MOTOR_LEFT_REAR_REVERSE);
        SetInvertedMotor(RobotDrive::kFrontRightMotor,
                         MOTOR_RIGHT_FRONT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearRightMotor,
                         MOTOR_RIGHT_REAR_REVERSE);

        SetSafetyEnabled(false);
        SetExpiration(1.0);

        RegisterTask("DriveBase", TASK_START_MODE | TASK_STOP_MODE);
    }   //DriveBase

    ~DriveBase(void)
    {
        UnregisterTask();
        SetSafetyEnabled(false);
        Stop();
    }   //~DriveBase

    void TaskStartMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            SetSafetyEnabled(true);
        }
    }   //TaskStartMode

    void TaskStopMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            SetSafetyEnabled(false);
            Stop();
        }
    }   //TaskStopMode
};  //class DriveBase
