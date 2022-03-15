#ifndef	RTS_H
#define	RTS_H

#include "string.h"
#include <arduino.h>

#include "i2c_eeprom.h"

#include "../../inc/MarlinConfig.h"

extern int power_off_type_yes;

/*********************************/
#define FHONE   (0x5A)
#define FHTWO   (0xA5)
#define FHLENG  (0x06)
#define TEXTBYTELEN     18
#define MaxFileNumber   20

#define	CEIconGrap      12
#define	FileNum	MaxFileNumber
#define	FileNameLen	TEXTBYTELEN
#define	RTS_UPDATE_INTERVAL 2000
#define	RTS_UPDATE_VALUE    (2*RTS_UPDATE_INTERVAL)

#define SizeofDatabuf		26

/*************Register and Variable addr*****************/
#define	RegAddr_W	0x80
#define	RegAddr_R	0x81
#define	VarAddr_W	0x82
#define	VarAddr_R	0x83
#define	ExchangePageBase    ((unsigned long)0x5A010000)     // the first page ID. other page = first page ID + relevant num;
#define	StartSoundSet       ((unsigned long)0x060480A0)     // 06,start-music; 04, 4 musics; 80, the volume value; 04, return value about music number.
#define	FONT_EEPROM      0

#define	FanOn            255
#define	FanOff           0

/*variable addr*/
/*variable addr*/
#define	ExchangepageAddr	0x0084
#define	SoundAddr		  	0x00A0
#define	StartIcon		  	0x1000
#define	FeedrateDisplay		0x1006
#define	Stopprint		  	0x1008
#define	Pauseprint			0x100A
#define	Resumeprint			0x100C
#define	PrintscheduleIcon	0x100E
#define	Timehour			  0x1010
#define	Timemin				  0x1012
#define	IconPrintstatus	0x1014
#define	Percentage			0x1016
#define	FanKeyIcon			0x101E

#define	HeatPercentIcon		0x1024

#define	NzBdSet				  0x1032
#define	NozzlePreheat		0x1034
#define	NozzleTemp			0x1036
#define	BedPreheat			0x103A
#define	Bedtemp				  0x103C

#define	AutoZeroIcon		0x1042
#define	AutoLevelMode		0x1045
#define	AutoZero			  0x1046
#define	DisplayXaxis		0x1048
#define	DisplayYaxis		0x104A
#define	DisplayZaxis		0x104C

#define	FilementUnit1		0x1054
#define	Exchfilement		0x1056
#define	FilementUnit2		0x1058
#define LCDKeyIcon			0x105A

#define	MacVersion			0x1060
#define	SoftVersion			0x106A
#define	PrinterSize			0x1074
#define	CorpWebsite			0x107E
#define	VolumeIcon			0x108A
#define	SoundIcon			  0x108C
#define	AutolevelIcon		0x108D
#define	ExchFlmntIcon		0x108E
#define	AutolevelVal		0x1100

#define	FilenameIcon		0x1200
#define	FilenameIcon1		0x1220
#define	Printfilename		0x2000
#define	SDFILE_ADDR			0x200A
#define	FilenamePlay		0x20D2
#define	FilenameChs			0x20D3
#define	Choosefilename	0x20D4
#define	FilenameCount		0x20DE
#define	FilenameNature	0x6003


#define MACHINE_TYPE      "Sermoon D1"
#define FIRMWARE_VERSION  "Ver 1.1.16"
#define HARDWARE_VERSION  "HW 4.3.1"
#define SCREEN_VERSION    "DWIN 1.1.14"
#define PRINT_SIZE        "280*260*310"
#define	CORP_WEBSITE_C	  "www.cxsw3d.com  "
#define	CORP_WEBSITE_E	  "www.creality.com"

/************struct**************/
typedef struct DataBuf
{
    unsigned char len;  
    unsigned char head[2];
    unsigned char command;
    unsigned long addr;
    unsigned long bytelen;
    unsigned short data[32];
    unsigned char reserv[4];
} DB;

typedef struct CardRecord
{
    int recordcount;
    int Filesum;
    unsigned long addr[FileNum];
    char Cardshowfilename[FileNum][FileNameLen];
    char Cardfilename[FileNum][FileNameLen];
}CRec;

class RTSUI {
  public:
    #if ENABLED(LCD_BED_LEVELING) && EITHER(PROBE_MANUALLY, MESH_BED_LEVELING)
      static bool wait_for_bl_move;
    #else
      static constexpr bool wait_for_bl_move = false;
    #endif

    static int16_t preheat_hotend_temp[2], preheat_bed_temp[2];
    static uint8_t preheat_fan_speed[2];
};

extern RTSUI rtsui;

class RTSSHOW {
  public:
    RTSSHOW();
    int RTS_RecData();
    void RTS_SDCardInit(void);
    void RTS_SDCardUpate(void);
    int RTS_CheckFilement(int);
    void RTS_SndData(void);
    void RTS_SndData(const String &, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(const char[], unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(char, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned char*, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(int, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(float, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned int,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SDcard_Stop();
    void RTS_HandleData();
    void RTS_Init();
    
    DB recdat;
    DB snddat;
  private:
    unsigned char databuf[SizeofDatabuf];
  };

extern RTSSHOW rtscheck;

#define	Addvalue	3
#define	PrintChoice_Value	(0+Addvalue)
#define	Zoffset_Value		  (3+Addvalue)
#define	Setting_Value		  (8+Addvalue)
#define	XYZEaxis_Value		(12+Addvalue)
#define	Filement_Value		(15+Addvalue)
#define	Language_Value		(18+Addvalue)
#define	Filename_Value		(22+Addvalue)

enum ReturnKeyAddr {
                      MainPage = 0x2001,         //
                      SelectFile = 0x2002,
                      PrintOptions = 0x2007,
                      TempMenu = 0x200C,
                      AutoTemp = 0x200E,
                      ManualTemp = 0x2010,
                      SettingsMenu = 0x2011,
                      LevelMode = 0x2012,
                      ChangeFilament = 0x2015,
                      MovePage = 0x2019,
                      LanguageOptions = 0x201D,
                      PwrOffContinue = 0x2025,
                      NoFilamentContinue = 0x2026,
                    };

enum PROC_COM {
                Printfile = 0, 
                Ajust, 
                Feedrate, 
                PrintChoice = PrintChoice_Value, 
                Zoffset=Zoffset_Value, 
                TempControl, 
                ManualSetTemp,
                Setting=Setting_Value, 
                ReturnBack, 
                Bedlevel, 
                Autohome, 
                XYZEaxis = XYZEaxis_Value, 
                Filement = Filement_Value,
                LanguageChoice = Language_Value, 
                No_Filement, 
                PwrOffNoF, 
                Volume, 
                Filename = Filename_Value
              };

const unsigned long Addrbuf[] = {
                                  0x1002, 0x1004, 0x1006, 0x1008, 0x100A, //0 - 4
                                  0x100C, 0x1026, 0x1030, 0x1032, 0x1034, //5 - 9
                                  0x103A, 0x103E, 0x1040, 0x1044, 0x1046, //10 - 14
                                  0x1048, 0x104A, 0x104C, 0x1054, 0x1056, //15 - 19
                                  0x1058, 0x105C, 0x105E, 0x105F, 0x1088, //20 - 24
                                  0
                                };

extern void RTSUpdate();
extern void RTSInit();

extern bool heat_flag;
extern bool home_flag;

extern char waitway;
extern char CardCheckStatus[2];
extern bool InforShowStatus;
extern unsigned char LanguageRecbuf;
extern unsigned char AxisPagenum;
extern bool AutohomeKey;
extern bool TPShowStatus;
extern bool AutoLevelStatus;
extern int Update_Time_Value;
extern bool PoweroffContinue;
extern char FilementStatus[2];
extern char commandbuf[30];
extern int PrintModeTime;
extern float zprobe_zoffset;

extern unsigned char G29_status;
extern char PrinterStatusKey[2];
// PrintStatue[0], 0 represent  to 43 page, 1 represent to 44 page
extern char PrintStatue[2];
extern bool PreheatStatus[];
extern unsigned char language_change;

extern void Set_Language(unsigned char num);
extern void Lcd_Select_Var(unsigned char Num,int BeginAddr,unsigned char TotalNum);


//以下为9国语言相关宏定义
/********************************************************************************************************/

/*************** Root ****************/
#define Root_Key                0x2050
#define Erase_Status            0x2055
#define ENSURE_KEY              0x2056

/************** variable addr ************/
#define START_PROCESS_ICON_VP       0x1000    //开机进度条

//图标变量 → 标题
/********* variable addr  Tittle *********/
#define PRINT_TITLE_CHAR_VP         0x1001  //打印
#define TEMP_TITLE_CHAR_VP          0x1002  //温度
#define SET_TEMP_TITLE_CHAR_VP      0x1003  //手动设温
#define AUX_LEVEL_TITLE_CHAR_VP     0x1004  //辅助调平
#define REFUL_TITLE_CHAR_VP         0x1005  //换料
#define HEATING_TITLE_CHAR_VP       0x1006  //正在加热中
#define AUTO_HOME_TITLE_CHAR_VP     0x1007  //回零
#define LANGUAGE_TITLE_CHAR_VP      0x1008  //语言
#define ABOUT_TITLE_CHAR_VP         0x1009  //关于
#define SETTINGS_TITLE_CHAR_VP      0x100A  //设置
#define FAN_TITLE_CHAR_VP           0x100B  //风扇
#define LEVEL_MODE_TITLE_CHAR_VP    0x100C  //调平模式

//图标变量 → 提示信息
/*********** variable addr  Signal *************/
#define ASSIST_LEVEL_SIGNAL_VP      0x1110  //辅助调平
#define CHANGE_SIGNAL_VP            0x1111  //更换耗材
#define AUTO_HOME_SIGNAL_VP         0x1112  //回零
#define AUTO_LEVEL_SIGNAL_VP        0x1113  //自动调平
#define PROCESSING_SIGNAL_VP        0x1114  //执行中

////************* GIF ************////
#define LEVEL_AUTORUN_VP            0x1120  //自动调平
#define HOME_AUTORUN_VP             0x1121  //自动回零

#define MESSAGE_WARING_VP           0x1122  //警告信息
#define REFUEL_GIF_FILEMENT_VP      0x1123  //换料动图

//字符图标变量
//主菜单
/*********** variable addr  Text **********/
#define PRINT_MAIN_CHAR_VP          0x1020  //打印
#define TEMP_MAIN_CHAR_VP           0x1021  //温度
#define SETTINGS_MAIN_CHAR_VP       0x1022  //设置
//打印
#define BEGIN_PRINT_CHAR_VP         0x1023  //开始
#define COMPLETE_PRINT_CHAR_VP      0x1024  //打印完成
#define STOP_PRINT_CHAR_VP          0x1025  //停止
#define ADJUST_PRINT_CHAR_VP        0x1029  //调整
#define PAUSE_PRINT_CHAR_VP         0x1027  //暂停
#define CONTINUE_PRINT_CHAR_VP      0x1028  //继续
#define PRINT_PERCENT_DATA_VP       0x1408  //百分比
//Z轴补偿
#define Z_OFFSET_Z_CHAR_VP          0x1030  //Z轴补偿:
#define UNIT_Z_CHAR_VP              0x1031  //单位 0.01mm
#define FAN_Z_CHAR_VP               0x1032  //风扇
#define PRINT_SPEED_Z_CHAR_VP       0x1033  //打印速度
#define NOZZLE_TEMP_Z_CHAR_VP       0x1034  //喷头温度
#define BED_TEMP_Z_CHAR_VP          0x1035  //热床温度
//温度
#define AUTO_TEMP_CHAR_VP           0x1040  //自动设温
#define MANUAL_TEMP_CHAR_VP         0x1041  //手动设温
#define COOLING_TEMP_CHAR_VP        0x1042  //冷却
#define FAN_TEMP_CHAR_VP            0x1043  //风扇
//手动设温
#define NOZZLE_PREHEAT_CHAR_VP      0x1050  //喷头预热
#define NOZZLE_TEMP_CHAR_VP         0x1051  //喷头温度
#define BED_PREHEAT_CHAR_VP         0x1052  //热床预热
#define BED_TEMP_CHAR_VP            0x1053  //热床温度
#define COOLING_MANUAL_CHAR_VP      0x1054  //一键降温
//换料页面
#define EXTRUDER_REFUL_CHAR_VP      0x1060  //挤出机
#define UNIT_REFUL_CHAR_VP          0x1061  //单位:mm
#define FEED1_REFUL_CHAR_VP         0x1062  //进料1
#define FEED2_REFUL_CHAR_VP         0x1063  //进料2
#define RETREAT1_REFUL_CHAR_VP      0x1064  //退料1
#define RETREAT2_REFUL_CHAR_VP      0x1065  //退料2
//关于页面
#define MACHINE_TYPE_ABOUT_CHAR_VP  0x1080  //设备型号
#define HW_VERSION_ABOUT_CHAR_VP    0x1081  //硬件版本
#define FW_VERSION_ABOUT_CHAR_VP    0x1082  //固件版本
#define DP_VERSION_ABOUT_CHAR_VP    0x1083  //显示屏版本
#define PRINT_SIZE_ABOUT_CHAR_VP    0x1084  //打印尺寸
#define WEBSITE_ABOUT_CHAR_VP       0x1085  //公司网站
//设置页面
#define LEVELING_SET_CHAR_VP        0x1090  //调平模式
#define REFUL_SET_CHAR_VP           0x1091  //换料
#define MOVE_SET_CHAR_VP            0x1092  //移动轴
#define MOTOR_CTRL_SET_CHAR_VP      0x1093  //电机控制
#define LANGUAGE_SET_CHAR_VP        0x1094  //语言
#define ABOUT_SET_CHAR_VP           0x1095  //关于
#define MOTOR_STATUS_ICON_VP        0x1224  //电机图标
//调平模式
#define MEASURE_LEVEL_CHAR_VP       0x10A0  //平台测量
#define AUX_LEVEL_CHAR_VP           0x10A1  //辅助调平
#define AUTO_LEVEL_CHAR_VP          0x10A2  //自动调平
//移动轴
#define UNIT_MOVE_CHAR_VP           0x10B0  //单位
#define X_MOVE_CHAR_VP              0x10B1  //X轴
#define Y_MOVE_CHAR_VP              0x10B2  //Y轴
#define Z_MOVE_CHAR_VP              0x10B3  //Z轴

//图标变量 → 对话框
/************ variable addr  Dialog ************/
#define MOTOR_OFF_DIALOG_VP         0x1100  //关闭电机
#define RESUME_PRINT_DIALOG_VP      0x1101  //断电续打
#define REFUEL_DIALOG_VP            0x1102  //换料警告
#define HEATING_DIALOG_VP           0x1103  //加热中
#define NO_FILAMENT_DIALOG_VP       0x1104  //材料耗尽
#define COOLING_DIALOG_VP           0x1105  //冷却确认
#define PAUSE_DIALOG_VP             0x1106  //暂停确认
#define STOP_DIALOG_VP              0x1107  //停止确认
#define CHOOSE_MODE_DIALOG_VP       0x1108  //选择模式
#define CANCEL_DIALOG_VP            0x1109  //取消
#define YES_1_DIALOG_VP             0x110A  //是-1
#define YES_2_DIALOG_VP             0x110B  //是-2
#define NO_1_DIALOG_VP              0x110C  //否-1
#define NO_2_DIALOG_VP              0x110D  //否-2


/********* variable addr  Status *********/
#define FILE_SELECT_1_ICON_VP       0x1200
#define FILE_SELECT_2_ICON_VP       0x1201
#define FILE_SELECT_3_ICON_VP       0x1202
#define FILE_SELECT_4_ICON_VP       0x1203
#define FILE_SELECT_5_ICON_VP       0x1204
#define FILE_SELECT_6_ICON_VP       0x1205
#define FILE_SELECT_7_ICON_VP       0x1206
#define FILE_SELECT_8_ICON_VP       0x1207
#define FILE_SELECT_9_ICON_VP       0x1208
#define FILE_SELECT_10_ICON_VP      0x1209
#define FILE_SELECT_11_ICON_VP      0x120A
#define FILE_SELECT_12_ICON_VP      0x120B
#define FILE_SELECT_13_ICON_VP      0x120C
#define FILE_SELECT_14_ICON_VP      0x120D
#define FILE_SELECT_15_ICON_VP      0x120E
#define FILE_SELECT_16_ICON_VP      0x120F
#define FILE_SELECT_17_ICON_VP      0x1210
#define FILE_SELECT_18_ICON_VP      0x1211
#define FILE_SELECT_19_ICON_VP      0x1212
#define FILE_SELECT_20_ICON_VP      0x1213

#define FAN_SWITCH_ICON_VP          0x1220
#define AUTOLEVEL_SWITCH_ICON_VP    0x1221
#define PLA_MODE_ICON_VP            0x1222
#define ABS_MODE_ICON_VP            0x1223


#define LANGUAGE_1_ICON_VP          0x1225
#define LANGUAGE_2_ICON_VP          0x1226
#define LANGUAGE_3_ICON_VP          0x1227
#define LANGUAGE_4_ICON_VP          0x1228
#define LANGUAGE_5_ICON_VP          0x1229
#define LANGUAGE_6_ICON_VP          0x122A
#define LANGUAGE_7_ICON_VP          0x122B
#define LANGUAGE_8_ICON_VP          0x122C
#define LANGUAGE_9_ICON_VP          0x122D

#define STATUS_DP_CHAR_VP           0x1300
/*
#define ABS_STATUS_CHAR_VP              0x1300
#define PLA_STATUS_CHAR_VP              0x1301
#define CARD_IN_STATUS_CHAR_VP          0x1302
#define CARD_OUT_STATUS_CHAR_VP         0x1303
#define READY_STATUS_CHAR_VP            0x1304
#define HEATING_STATUS_CHAR_VP          0x1305
#define HEATING_DONE_STATUS_CHAR_VP     0x1306
#define PRINTING_STATUS_CHAR_VP         0x1307
#define PRINTING_DONE_STATUS_CHAR_VP    0x1308
#define COOLING_STATUS_CHAR_VP          0x1309
#define COOL_DOWN_STATUS_CHAR_VP        0x130A
#define PAUSE_STATUS_CHAR_VP            0x130B
#define ENERGY_SAVING_ON_CHAR_VP        0x130C
#define ENERGY_SAVING_OFF_CHAR_VP       0x130D
*/

/*********** Data variables ****************/

#define NOZZLE_PREHEAT_DATA_VP      0x1400  //喷头预热温度
#define NOZZLE_TEMP_DATA_VP         0x1402  //碰头当前温度
#define BED_PREHEAT_DATA_VP         0x1404  //热床预热温度
#define BED_TEMP_DATA_VP            0x1406  //热床当前温度

#define PRINT_TIMEHOUR_DATA_VP      0x140B
#define PRINT_TIMEMIN_DATA_VP       0x140E

#define PRINT_SPEED_DATA_VP         0x1414

#define CHANGE_FILAMENT_DATA_VP     0x1418
#define CHANGE_FILAMENT_UNIT_KEY    0x1418

#define AUTO_LEVEL_DATA_VP          0x1840


#define Z_OFFSET_DATA_VP            0x2100
#define PRINT_SPEED_KEY             0x1414

#define TEMP_WARNING_DATA_VP        0x2120


#define X_MOVE_DATA_KEY             0x2112
#define Y_MOVE_DATA_KEY             0x2114
#define Z_MOVE_DATA_KEY             0x2116


/*--------------- Text ---------------*/
#define FILE_FIRST_TEXT_VP      0x1600

#define FILE_DISPLAY_1_TEXT_VP  0x1600
#define FILE_DISPLAY_2_TEXT_VP  0x1614
#define FILE_DISPLAY_3_TEXT_VP  0x1628
#define FILE_DISPLAY_4_TEXT_VP  0x163C
#define FILE_DISPLAY_5_TEXT_VP  0x1650
#define FILE_DISPLAY_6_TEXT_VP  0x1664
#define FILE_DISPLAY_7_TEXT_VP  0x1678
#define FILE_DISPLAY_8_TEXT_VP  0x168C
#define FILE_DISPLAY_9_TEXT_VP  0x16A0
#define FILE_DISPLAY_10_TEXT_VP 0x16B4
#define FILE_DISPLAY_11_TEXT_VP 0x16C8
#define FILE_DISPLAY_12_TEXT_VP 0x16DC
#define FILE_DISPLAY_13_TEXT_VP 0x16F0
#define FILE_DISPLAY_14_TEXT_VP 0x1704
#define FILE_DISPLAY_15_TEXT_VP 0x1718
#define FILE_DISPLAY_16_TEXT_VP 0x172C
#define FILE_DISPLAY_17_TEXT_VP 0x1740
#define FILE_DISPLAY_18_TEXT_VP 0x1754
#define FILE_DISPLAY_19_TEXT_VP 0x1768
#define FILE_DISPLAY_20_TEXT_VP 0x177C

#define FILE_SELECTED_TEXT_VP   0x1790

#define MACHINE_TYPE_TEXT_VP    0x17B0
#define HW_VERSION_TEXT_VP      0x17C4
#define FW_VERSION_TEXT_VP      0x17D8
#define DP_VERSION_TEXT_VP      0x17EC
#define PRINT_SIZE_TEXT_VP      0x1800
#define WEBSITE_TEXT_VP         0x1814



#endif// RTS_H
