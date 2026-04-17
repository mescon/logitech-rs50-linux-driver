using System;
using System.Collections;
using System.Collections.Specialized;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Text;

public class LogitechGSDK
{
    //STEERING WHEEL SDK
    public const int LOGI_MAX_CONTROLLERS = 4;

    //Force types

    public const int LOGI_FORCE_NONE = -1;
    public const int LOGI_FORCE_SPRING = 0;
    public const int LOGI_FORCE_CONSTANT = 1;
    public const int LOGI_FORCE_DAMPER = 2;
    public const int LOGI_FORCE_SIDE_COLLISION = 3;
    public const int LOGI_FORCE_FRONTAL_COLLISION = 4;
    public const int LOGI_FORCE_DIRT_ROAD = 5;
    public const int LOGI_FORCE_BUMPY_ROAD = 6;
    public const int LOGI_FORCE_SLIPPERY_ROAD = 7;
    public const int LOGI_FORCE_SURFACE_EFFECT = 8;
    public const int LOGI_NUMBER_FORCE_EFFECTS = 9;
    public const int LOGI_FORCE_SOFTSTOP = 10;
    public const int LOGI_FORCE_CAR_AIRBORNE = 11;


    //Periodic types  for surface effect

    public const int LOGI_PERIODICTYPE_NONE = -1;
    public const int LOGI_PERIODICTYPE_SINE = 0;
    public const int LOGI_PERIODICTYPE_SQUARE = 1;
    public const int LOGI_PERIODICTYPE_TRIANGLE = 2;


    //Devices types

    public const int LOGI_DEVICE_TYPE_NONE = -1;
    public const int LOGI_DEVICE_TYPE_WHEEL = 0;
    public const int LOGI_DEVICE_TYPE_JOYSTICK = 1;
    public const int LOGI_DEVICE_TYPE_GAMEPAD = 2;
    public const int LOGI_DEVICE_TYPE_OTHER = 3;
    public const int LOGI_NUMBER_DEVICE_TYPES = 4;


    //Manufacturer types

    public const int LOGI_MANUFACTURER_NONE = -1;
    public const int LOGI_MANUFACTURER_LOGITECH = 0;
    public const int LOGI_MANUFACTURER_MICROSOFT = 1;
    public const int LOGI_MANUFACTURER_OTHER = 2;


    //Model types

    public const int LOGI_MODEL_G27 = 0;
    public const int LOGI_MODEL_DRIVING_FORCE_GT = 1;
    public const int LOGI_MODEL_G25 = 2;
    public const int LOGI_MODEL_MOMO_RACING = 3;
    public const int LOGI_MODEL_MOMO_FORCE = 4;
    public const int LOGI_MODEL_DRIVING_FORCE_PRO = 5;
    public const int LOGI_MODEL_DRIVING_FORCE = 6;
    public const int LOGI_MODEL_NASCAR_RACING_WHEEL = 7;
    public const int LOGI_MODEL_FORMULA_FORCE = 8;
    public const int LOGI_MODEL_FORMULA_FORCE_GP = 9;
    public const int LOGI_MODEL_FORCE_3D_PRO = 10;
    public const int LOGI_MODEL_EXTREME_3D_PRO = 11;
    public const int LOGI_MODEL_FREEDOM_24 = 12;
    public const int LOGI_MODEL_ATTACK_3 = 13;
    public const int LOGI_MODEL_FORCE_3D = 14;
    public const int LOGI_MODEL_STRIKE_FORCE_3D = 15;
    public const int LOGI_MODEL_G940_JOYSTICK = 16;
    public const int LOGI_MODEL_G940_THROTTLE = 17;
    public const int LOGI_MODEL_G940_PEDALS = 18;
    public const int LOGI_MODEL_RUMBLEPAD = 19;
    public const int LOGI_MODEL_RUMBLEPAD_2 = 20;
    public const int LOGI_MODEL_CORDLESS_RUMBLEPAD_2 = 21;
    public const int LOGI_MODEL_CORDLESS_GAMEPAD = 22;
    public const int LOGI_MODEL_DUAL_ACTION_GAMEPAD = 23;
    public const int LOGI_MODEL_PRECISION_GAMEPAD_2 = 24;
    public const int LOGI_MODEL_CHILLSTREAM = 25;
    public const int LOGI_MODEL_G29 = 26;
    public const int LOGI_MODEL_G920 = 27;
    public const int LOGI_NUMBER_MODELS = 28;

    [StructLayout(LayoutKind.Sequential, Pack = 2)]
    public struct LogiControllerPropertiesData
    {
        public bool forceEnable;
        public int overallGain;
        public int springGain;
        public int damperGain;
        public bool defaultSpringEnabled;
        public int defaultSpringGain;
        public bool combinePedals;
        public int wheelRange;
        public bool gameSettingsEnabled;
        public bool allowGameSettings;
    }


    [StructLayout(LayoutKind.Sequential, Pack = 2)]
    public struct DIJOYSTATE2ENGINES
    {
        public int lX;              /* x-axis position              */
        public int lY;              /* y-axis position              */
        public int lZ;              /* z-axis position              */
        public int lRx;             /* x-axis rotation              */
        public int lRy;             /* y-axis rotation              */
        public int lRz;             /* z-axis rotation              */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public int[] rglSlider;     /* extra axes positions         */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public uint[] rgdwPOV;      /* POV directions               */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)]
        public byte[] rgbButtons;   /* 128 buttons                  */
        public int lVX;             /* x-axis velocity              */
        public int lVY;             /* y-axis velocity              */
        public int lVZ;             /* z-axis velocity              */
        public int lVRx;            /* x-axis angular velocity      */
        public int lVRy;            /* y-axis angular velocity      */
        public int lVRz;            /* z-axis angular velocity      */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public int[] rglVSlider;    /* extra axes velocities        */
        public int lAX;             /* x-axis acceleration          */
        public int lAY;             /* y-axis acceleration          */
        public int lAZ;             /* z-axis acceleration          */
        public int lARx;            /* x-axis angular acceleration  */
        public int lARy;            /* y-axis angular acceleration  */
        public int lARz;            /* z-axis angular acceleration  */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public int[] rglASlider;    /* extra axes accelerations     */
        public int lFX;             /* x-axis force                 */
        public int lFY;             /* y-axis force                 */
        public int lFZ;             /* z-axis force                 */
        public int lFRx;            /* x-axis torque                */
        public int lFRy;            /* y-axis torque                */
        public int lFRz;            /* z-axis torque                */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public int[] rglFSlider;    /* extra axes forces            */
    };

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiSteeringInitialize(bool ignoreXInputControllers);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiUpdate();

    [DllImport("LogitechSteeringWheelEnginesWrapper", CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr LogiGetStateENGINES(int index);

    public static DIJOYSTATE2ENGINES LogiGetStateCSharp(int index)
    {
        DIJOYSTATE2ENGINES ret = new DIJOYSTATE2ENGINES();
        ret.rglSlider = new int[2];
        ret.rgdwPOV = new uint[4];
        ret.rgbButtons = new byte[128];
        ret.rglVSlider = new int[2];
        ret.rglASlider = new int[2];
        ret.rglFSlider = new int[2];
        try
        {
            ret = (DIJOYSTATE2ENGINES)Marshal.PtrToStructure(LogiGetStateENGINES(index), typeof(DIJOYSTATE2ENGINES));
        }
        catch (System.ArgumentException)
        {
            Debug.Write("Exception catched");
        }
        return ret;
    }

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiGetDevicePath(int index, StringBuilder str, int size);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiGetFriendlyProductName(int index, StringBuilder str, int size);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiIsConnected(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiIsDeviceConnected(int index, int deviceType);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiIsManufacturerConnected(int index, int manufacturerName);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiIsModelConnected(int index, int modelName);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiButtonTriggered(int index, int buttonNbr);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiButtonReleased(int index, int buttonNbr);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiButtonIsPressed(int index, int buttonNbr);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiGenerateNonLinearValues(int index, int nonLinCoeff);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern int LogiGetNonLinearValue(int index, int inputValue);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiHasForceFeedback(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiIsPlaying(int index, int forceType);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlaySpringForce(int index, int offsetPercentage, int saturationPercentage, int coefficientPercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopSpringForce(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlayConstantForce(int index, int magnitudePercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopConstantForce(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlayDamperForce(int index, int coefficientPercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopDamperForce(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlaySideCollisionForce(int index, int magnitudePercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlayFrontalCollisionForce(int index, int magnitudePercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlayDirtRoadEffect(int index, int magnitudePercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopDirtRoadEffect(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlayBumpyRoadEffect(int index, int magnitudePercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopBumpyRoadEffect(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlaySlipperyRoadEffect(int index, int magnitudePercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopSlipperyRoadEffect(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlaySurfaceEffect(int index, int type, int magnitudePercentage, int period);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopSurfaceEffect(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlayCarAirborne(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopCarAirborne(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlaySoftstopForce(int index, int usableRangePercentage);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiStopSoftstopForce(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiSetPreferredControllerProperties(LogiControllerPropertiesData properties);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiGetCurrentControllerProperties(int index, ref LogiControllerPropertiesData properties);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern int LogiGetShifterMode(int index);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiSetOperatingRange(int index, int range);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiGetOperatingRange(int index, ref int range);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool LogiPlayLeds(int index, float currentRPM, float rpmFirstLedTurnsOn, float rpmRedLine);

    [DllImport("LogitechSteeringWheelEnginesWrapper", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
    public static extern void LogiSteeringShutdown();
}


