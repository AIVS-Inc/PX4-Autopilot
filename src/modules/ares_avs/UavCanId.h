/*
 * \file UavCanId.h
 */

#ifndef CAN_ARES_UAVCANID_H_
#define CAN_ARES_UAVCANID_H_

////////////////////////
// Defined Subject Ids, must change to full dynamic Cyphal register defined Ids
////////////////////////
#define ARES_SUBJECT_ID_GNSS_PARAMS				509
#define ARES_SUBJECT_ID_PHASOR_PARAMS				508
#define ARES_SUBJECT_ID_CAN_PARAMS				507
#define ARES_SUBJECT_ID_ADC_PARAMS				506
#define ARES_SUBJECT_ID_ADC_SYNC				505
#define ARES_SUBJECT_ID_STORAGE_PARAMS				504
#define ARES_SUBJECT_ID_STORAGE_CONTROL				503
#define ARES_SUBJECT_ID_CLOCK_PARAMS				499
#define ARES_SUBJECT_ID_CLOCK_CONTROL				502
#define ARES_SUBJECT_ID_FFT_PARAMS				501
#define ARES_SUBJECT_ID_FFT_CONTROL				500
#define ARES_SUBJECT_ID_CONSOLE_INPUT				499
#define ARES_SUBJECT_ID_CONSOLE_CONTROL				498
#define ARES_SUBJECT_ID_LED_CONTROL				497
#define ARES_SUBJECT_ID_ACCELEROMETER_CONTROL		 	496

#define ARES_SUBJECT_ID_GNSS_RELPOSNED				1234
#define ARES_SUBJECT_ID_PHASOR_FIELD				1235
#define ARES_SUBJECT_ID_GNSS_POSITION				1236
#define ARES_SUBJECT_ID_GNSS_RTCM				1237
#define ARES_SUBJECT_ID_WX_WX					1238
#define ARES_SUBJECT_ID_WX_WIND					1239
#define ARES_SUBJECT_ID_PHASOR_PROXY				1240
#define ARES_SUBJECT_ID_FFT_FULL_SPECTRUM			1241
#define ARES_SUBJECT_ID_FFT_INTENSITY				1242
#define ARES_SUBJECT_ID_FFT_BEARING_ANGLES			1243
#define ARES_SUBJECT_ID_FFT_ADC_FRAME				1244
#define ARES_SUBJECT_ID_CONSOLE_OUTPUT				1245
#define ARES_SUBJECT_ID_FFT_RPM					1246
#define ARES_SUBJECT_ID_FFT_MEL_INTENSITY			1247

////////////////////////
// ARES FFT Parameters
////////////////////////
#define ARES_FFT_PARAMS_EXTENT	32
typedef enum	// ares_fft_ParamId
{
	ares_fft_ParamId_Min = 0,
	ares_fft_ParamId_OutputDecimator = ares_fft_ParamId_Min,
	ares_fft_ParamId_DataType,
	ares_fft_ParamId_FrequencyResolution,
	ares_fft_ParamId_EncryptOutput,
	ares_fft_ParamId_AutoStart,
	ares_fft_ParamId_Length,
	ares_fft_ParamId_Linear,
	ares_fft_ParamId_PeakDetector,
	ares_fft_ParamId_HannWindow,
	ares_fft_ParamId_EventDetector,
	ares_fft_ParamId_COUNT,
	ares_fft_ParamId_Invalid,
	ares_fft_ParamId_Max = ares_fft_ParamId_COUNT - 1,
} ares_fft_ParamId;

typedef struct __attribute__(( packed )) // ares_fft_Param_OutputDecimator
{
	uint8_t m_u8Decimator;
	uint8_t m_u8TxDelay;
} ares_fft_Param_OutputDecimator;

typedef struct __attribute__(( packed )) // ares_fft_Param_Length
{
	uint16_t m_u16NumberOfBins;
	uint16_t m_u16NumberOfBlocks;
} ares_fft_Param_Length;

typedef struct __attribute__(( packed )) // ares_fft_Param_Linear
{
	uint16_t m_u16StartingBin;
	uint16_t m_u16NumberOfBins;
} ares_fft_Param_Linear;

typedef struct __attribute__(( packed )) // ares_fft_Param_PeakDetector
{
	float m_fStartingFrequency;
	int m_iNumberOfPeaks;
	float m_fMinimumPeakHeight;
	float m_fMinimumPeakChange;
	float m_fMinimumPeakBlanking;
} ares_fft_Param_PeakDetector;

typedef struct __attribute__(( packed )) // ares_fft_Param_HannWindow
{
	uint8_t m_u8Enable;
} ares_fft_Param_HannWindow;

typedef struct __attribute__(( packed )) // ares_fft_Param_EventDetector
{
	float m_fRelativeDb;
	uint16_t m_iNumberOfSources;
	uint8_t m_iAngularResln;
	uint16_t m_iBgTimeConstant;
	uint8_t m_iEventWindow;
	bool m_bSelfMeasureBg;
	float m_fBgDbThreshold;

} ares_fft_Param_EventDetector;

typedef struct __attribute__(( packed, aligned(2)))	// ares_fft_Param_0_1
{
	uint16_t m_u16ParamId;
	uint8_t m_u8ParamValue[ 0 ];
} ares_fft_Param_0_1;


typedef enum	// ares_fft_ControlId
{
	FftControlId_Enable,
	FftControlId_ReplayFile,
	FftControlId_CalibrateFromFile,
	FftControlId_Result,
	FftControlId_COUNT,
	FftControlId_Invalid,
	FftControlId_Min = 0,
	FftControlId_Max = FftControlId_COUNT - 1,
} ares_fft_ControlId;

typedef struct __attribute__(( aligned( 4 ) ))	// ares_fft_Control_0_1
{
	uint16_t m_u16ControlId;
	uint16_t m_u16Length;
	uint8_t m_au8Data[ 0 ];
} ares_fft_Control_0_1;

////////////////////////
// ARES Storage Parameters
////////////////////////
typedef enum	// ares_storage_ControlId
{
	StorageControlId_Min = 0,
	StorageControlId_CardMount = StorageControlId_Min,		// and unmount
	StorageControlId_CardStatus,	// card size, free space, status
	StorageControlId_FileStart,		// and stop
	StorageControlId_FileStatus,	// file size/status
	StorageControlId_FileRead,
	StorageControlId_FileDelete,
	StorageControlId_DirectoryList,
	StorageControlId_AppendFileOpen,
	StorageControlId_AppendFileWrite,
	StorageControlId_AppendFileClose,
	StorageControlId_GetFileSize,
	StorageControlId_ACK,
	StorageControlId_NACK,
	StorageControlId_COUNT,
	StorageControlId_Invalid,
	StorageControlId_Max = StorageControlId_COUNT - 1,
} ares_storage_ControlId;

typedef struct __attribute__(( packed, aligned( 4 ) )) // ares_storage_Control_0_1
{
	uint16_t m_u16ControlId;
	uint16_t m_u16Length;
	uint8_t m_au8Data[ 0 ];
} ares_storage_Control_0_1;

////////////////////////
// ARES GNSS Parameters
////////////////////////
typedef enum	// GnssParamId
{
	GnssParamId_None,
	GnssParamId_PositionPeriod,
	GnssParamId_RelPosNedPeriod,
	GnssParamId_RtcmMode,
	GnssParamId_COUNT,
	GnssParamId_Invalid,
	GnssParamId_Min = 0,
	GnssParamId_Max = GnssParamId_COUNT - 1,
} GnssParamId;

typedef enum	// GnssRtcmMode
{
	GnssRtcmMode_None,
	GnssRtcmMode_Rover,
	GnssRtcmMode_Base,
	GnssRtcmMode_MovingBase,
	GnssRtcmMode_COUNT,
	GnssRtcmMode_Invalid,
	GnssRtcmMode_Min = 0,
	GnssRtcmMode_Max = GnssRtcmMode_COUNT - 1,
} GnssRtcmMode;

typedef struct	// GnssParam
{
	GnssParamId m_gpiGnssParamId;
	union
	{
		size_t m_uPositionPeriod;
		size_t m_uRelPosNedPeriod;
		GnssRtcmMode m_grmRtcmMode;
	};
} GnssParam;

typedef enum	// GnssRtcmOptions
{
	GnssRtcmOptions_None = 0,
	GnssRtcmOptions_Rover = ( 1 << 0 ),
	GnssRtcmOptions_Base = ( 1 << 1 ),
} GnssRtcmOptions;

typedef enum	// GnssInfoOptions
{
	GnssInfoOptions_None = 0,
	GnssInfoOptions_Error = ( 1 << 0 ),
	GnssInfoOptions_Warning = ( 1 << 2 ),
	GnssInfoOptions_Notice = ( 1 << 3 ),
	GnssInfoOptions_Test = ( 1 << 4 ),
	GnssInfoOptions_Debug = ( 1 << 5 ),
} GnssInfoOptions;

typedef enum	// GnssSurveyMode
{
	GnssSurveyMode_Disabled = 0,
	GnssSurveyMode_SurveyIn = 1,
	GnssSurveyMode_Fixed = 2,
	GnssSurveyMode_COUNT,
	GnssSurveyMode_Invalid,
	GnssSurveyMode_Min = 0,
	GnssSurveyMode_Max = GnssSurveyMode_COUNT - 1,
} GnssSurveyMode;

typedef struct	// GnssSurveyInfo
{
	GnssSurveyMode m_gsmGnssSurveyMode;
	bool m_bSurveyActive;
	bool m_bSurveyValid;
	uint32_t m_u32ElapsedSeconds;
	float m_fMeanAccuracy;
} GnssSurveyInfo;

typedef enum	// GnssSubscriptionOptions
{
	GnssSubscriptionOptions_None = 0,
	GnssSubscriptionOptions_Time = ( 1 << 0 ),
	GnssSubscriptionOptions_Position = ( 1 << 1 ),
	GnssSubscriptionOptions_Rtcm = ( 1 << 2 ),
	GnssSubscriptionOptions_SurveyStatus = ( 1 << 3 ),
	GnssSubscriptionOptions_RelPosNed= ( 1 << 4 ),
} GnssSubscriptionOptions;

typedef enum	// GnssCarrierPhaseStatus
{
	GnssCarrierPhaseStatus_None = 0,
	GnssCarrierPhaseStatus_Floating = 1,
	GnssCarrierPhaseStatus_Fixed = 2,
} GnssCarrierPhaseStatus;

typedef struct	// GnssParams
{
	size_t m_uPositionPeriod;	// seconds
	size_t m_uRelPosNedPeriod;
	GnssRtcmOptions m_groRtcmOptions;
} GnssParams;

typedef struct __attribute__((aligned(2)))	// ares_gnss_Param_0_1
{
	uint16_t m_u16ParamId;
	uint8_t m_u8ParamValue[ 0 ];
} ares_gnss_Param_0_1;


////////////////////////
// ARES ADC Sync Parameters
////////////////////////
typedef struct __attribute__((aligned(4)))	// ares_adc_Sync_0_1
{
	int64_t m_i64UtcSecond;
} ares_adc_Sync_0_1;


#endif /* CAN_ARES_UAVCANID_H_ */
