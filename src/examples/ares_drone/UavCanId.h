/*
 * \file UavCanId.h
 */

#ifndef CAN_ARES_UAVCANID_H_
#define CAN_ARES_UAVCANID_H_

#define ARES_SUBJECT_ID_GNSS_PARAMS				 509
#define ARES_SUBJECT_ID_PHASOR_PARAMS				 508
#define ARES_SUBJECT_ID_CAN_PARAMS				 507
#define ARES_SUBJECT_ID_ADC_PARAMS				 506
#define ARES_SUBJECT_ID_ADC_SYNC				 505
#define ARES_SUBJECT_ID_STORAGE_PARAMS				 504
#define ARES_SUBJECT_ID_STORAGE_CONTROL				 503
#define ARES_SUBJECT_ID_CLOCK_PARAMS				 499
#define ARES_SUBJECT_ID_CLOCK_CONTROL				 502
#define ARES_SUBJECT_ID_FFT_PARAMS				 501
#define ARES_SUBJECT_ID_FFT_CONTROL				 500
#define ARES_SUBJECT_ID_CONSOLE_INPUT				 499
#define ARES_SUBJECT_ID_CONSOLE_CONTROL				 498
#define ARES_SUBJECT_ID_LED_CONTROL				 497
#define ARES_SUBJECT_ID_ACCELEROMETER_CONTROL		 	 496
#define ARES_SUBJECT_ID_RPM_CONTROL				 497

#define ARES_SUBJECT_ID_PHASOR_FIELD				1235
#define ARES_SUBJECT_ID_GNSS_POSITION				1236
#define ARES_SUBJECT_ID_GNSS_RTCM				1237
#define ARES_SUBJECT_ID_GNSS_RELPOSNED				1234
#define ARES_SUBJECT_ID_WX_WX					1238
#define ARES_SUBJECT_ID_WX_WIND					1239
#define ARES_SUBJECT_ID_PHASOR_PROXY				1240
#define ARES_SUBJECT_ID_FFT_FULL_SPECTRUM			1241
#define ARES_SUBJECT_ID_FFT_INTENSITY				1242
#define ARES_SUBJECT_ID_FFT_BEARING_ANGLES			1243
#define ARES_SUBJECT_ID_FFT_ADC_FRAME				1244
#define ARES_SUBJECT_ID_CONSOLE_OUTPUT				1245

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
	ares_fft_ParamId_EventParam,
	ares_fft_ParamId_HannWindow,
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
    	float m_relativeDb;
    	uint32_t m_numSources;
    	float m_angularRes;
    	float m_bkgndSILtc;
    	float m_eventWindow;
} ares_fft_Param_EventParam;

typedef struct __attribute__(( packed )) // ares_fft_Param_HannWindow
{
	uint8_t m_u8Enable;
} ares_fft_Param_HannWindow;

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

#endif /* CAN_ARES_UAVCANID_H_ */
