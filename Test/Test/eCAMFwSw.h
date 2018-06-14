// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the ECAMFWSW_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// ECAMFWSW_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef ECAMFWSW_EXPORTS
#define ECAMFWSW_API __declspec(dllexport)
#else
#define ECAMFWSW_API __declspec(dllimport)
#endif
					                   
#define ERROR_UVC_COMM_FAILED				0X20000005	 //UVC Chip communication failed.
#define ERROR_INVALID_HANDLE_VALUE			0X20000010 
#define ERROR_INVALID_EXCEPTION				0X20000011 

#define VID					L"2560"
#define See3CAM_STEREO		L"C114"

typedef struct {
	UINT16 IMU_VALUE_ID;
    double accX;
    double accY;
    double accZ;
	double gyroX;
	double gyroY;
	double gyroZ;
} IMUDATAOUTPUT_TypeDef;

typedef struct {
    INT8 IMU_UPDATE_MODE;
	UINT16 IMU_NUM_OF_VALUES;
} IMUDATAINPUT_TypeDef;


/* IMU VALUE UPDATE MODE */
#define IMU_CONT_UPDT_EN					 (0x01)
#define IMU_CONT_UPDT_DIS					 (0x02)

typedef struct {
	INT8 IMU_MODE;
    INT8 ACC_AXIS_CONFIG;
    INT8 ACC_SENSITIVITY_CONFIG;
	INT8 GYRO_AXIS_CONFIG;
    INT8 GYRO_SENSITIVITY_CONFIG;
	INT8 IMU_ODR_CONFIG;
} IMUCONFIG_TypeDef;

/* EXPOSURE CONTROL */
#define SEE3CAM_STEREO_EXPOSURE_MIN			(10)
#define SEE3CAM_STEREO_EXPOSURE_MAX			(1000000)
#define SEE3CAM_STEREO_EXPOSURE_DEF			(8000)

/* IMU MODE */
#define IMU_ACC_GYRO_DISABLE				(0x00)
#define IMU_ACC_ENABLE						(0x01)
#define IMU_ACC_GYRO_ENABLE					(0x03)


/* ACC AXIS CONTROL */
#define IMU_ACC_X_Y_Z_ENABLE				(0x07)
#define IMU_ACC_X_ENABLE					(0x01)
#define IMU_ACC_Y_ENABLE					(0x02)
#define IMU_ACC_Z_ENABLE					(0x04)

/* ACC ODR CONTROL */
#define IMU_ODR_10_14_9HZ					(0x01)
#define IMU_ODR_50_59_9HZ					(0x02)
#define IMU_ODR_119HZ						(0x03)
#define IMU_ODR_238HZ						(0x04)
#define IMU_ODR_476HZ						(0x05)
#define IMU_ODR_952HZ						(0x06)

/* ACC SENSITIVITY CONTROL */
#define IMU_ACC_SENS_2G						(0x00)
#define IMU_ACC_SENS_4G						(0x02)
#define IMU_ACC_SENS_8G						(0x03)
#define IMU_ACC_SENS_16G					(0x01)

/* GYRO AXIS CONTROL */
#define IMU_GYRO_X_Y_Z_ENABLE				(0x07)
#define IMU_GYRO_X_ENABLE					(0x01)
#define IMU_GYRO_Y_ENABLE					(0x02)
#define IMU_GYRO_Z_ENABLE					(0x04)

/* GYRO SENSITIVITY CONTROL */
#define IMU_GYRO_SENS_245DPS				(0x00)
#define IMU_GYRO_SENS_500DPS				(0x01)
#define IMU_GYRO_SENS_2000DPS				(0x03)

/* IMU VALUES CONTROL */
#define IMU_AXES_VALUES_MIN					(1)
#define IMU_AXES_VALUES_MAX					(65535)

//Inits the Extension unit of the Instance ID passed
BOOL InitExtensionUnit(TCHAR *USBInstanceID);

//Deinits the Extension unit 
BOOL DeinitExtensionUnit(void);

//Reads the Firmware version of the device.
BOOL ReadFirmwareVersion(UINT8 *pMajorVersion, UINT8 *pMinorVersion1, UINT16 *pMinorVersion2, UINT16 *pMinorVersion3);

//Reads the unique ID of the camera
BOOL GetCameraUniqueID(TCHAR *tszUniqueID);

//Outputs the current Exposure Value of the camera
BOOL GetManualExposureStereo(INT32 *ManualExposureValue);

//Sets the Exposure Value passed to the camera
BOOL SetManualExposureStereo(INT32 ManualExposureValue);

//Sets the Camera to Auto Exposure
BOOL SetAutoExposureStereo();

//Reads the current configuration of the IMU
BOOL GetIMUConfig(IMUCONFIG_TypeDef *lIMUConfig);

//Sets the current configuration of the IMU
BOOL SetIMUConfig(IMUCONFIG_TypeDef lIMUConfig);

//Configures the IMU to read the IMU data in a specific format
BOOL ControlIMUCapture(IMUDATAINPUT_TypeDef *lIMUInput);

//Reads the IMU values
BOOL GetIMUValueBuffer(HANDLE lIMUDataReadyEvent , IMUDATAOUTPUT_TypeDef *lIMUAxes);

//Reads back the Intrinsic and Extrinsic values of the camera from the flash
BOOL StereoCalibRead(unsigned char **IntrinsicBuffer, unsigned char **ExtrinsicBuffer, int *lIntFileLength, int *lExtFileLength);

//Reads the mode in which the camera is set
BOOL GetStreamModeStereo(UINT *iStreamMode);

//Sets the mode passed in the camera 
BOOL SetStreamModeStereo(UINT iStreamMode);

//Reads the temperature data of the IMU unit
BOOL GetIMUTemperatureData(UINT8 *MSBTemp, UINT8 *LSBTemp);

//Enables/Disables the HDR mode
BOOL SetHDRModeStereo(UINT HDRMode);

//Reads the status of the HDR mode
BOOL GetHDRModeStereo(UINT *HDRMode);