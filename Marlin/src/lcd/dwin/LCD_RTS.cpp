#include "LCD_RTS.h"
#include <wstring.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <libmaple/usart.h>

#include "../../Marlin.h"
#include "../../inc/MarlinConfig.h"
#include "../../module/configuration_store.h"
#include "../../core/serial.h"
#include "../../core/macros.h"

#include "../fontutils.h"
#include "../../sd/cardreader.h"
#include "../../feature/power_loss_recovery.h"
#include "../../feature/babystep.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"

#include "../../feature/bedlevel/abl/abl.h"

#ifdef RTS_AVAILABLE

  //#define CHECKFILEMENT false
  #define CHECKFILEMENT true

  bool state_stop_flag = 0;

  bool heat_flag = 0;
  bool home_flag = 0;

  bool flag_test = true;

  float zprobe_zoffset;
  float last_zoffset = 0.0;

  const float manual_feedrate_mm_m[] = { 100*60, 100*60, 15*60, 2*60 };

  int startprogress = 0;
  CRec CardRecbuf;
  int temphot = 0;
  int tempbed = 0;
  float pause_z = 0;
  float pause_e = 0;

  unsigned char Motor_Switch = 1;

  float PLA_ABSModeTemp = 195;
  millis_t next_rts_update_ms = 0;
  int last_target_temperature_bed;
  int last_target_temperature[4] = {0};
  enum{
    MSG_ABS_MODE,
    MSG_PLA_MODE,
    MSG_CARD_INSERT,
    MSG_CARD_PULLOUT,
    MSG_READY,
    MSG_HEATING,
    MSG_HEAT_FINISH,
    MSG_PRINTING,
    MSG_PRINT_OK,
    MSG_COOLING,
    MSG_COLL_FINISH,
    MSG_STOP,
    MSG_OPEN_ENERGY_MODE,
    MSG_CLOSE_ENERGY_MODE,
  };
  int msgStatus = MSG_READY;
  char waitway = 0;
  int recnum = 0;
  unsigned char Percentrecord = 0;
  float ChangeMaterialbuf[2] = {0};
  char NozzleTempStatus[3] = {0};
  
  int  PrintModeTime = 0;
  //bool PrintMode = true;
  bool PrintMode = false;  //关闭节能模式，不使用
  bool PrintModeChange = true;

  // PrintStatue[0], 0 represent  to 43 page, 1 represent to 44 page
  char PrintStatue[2] = {0};

  bool pause_action_flag = 0;

  int power_off_type_yes = 0;
  int power_off_continue = 0;

  // represents to update file list
  bool CardUpdate = false;

  // CardCheckStatus[0] represents to check card in printing and after making sure to begin and to check card in heating with value as 1, but 0 for off
  char CardCheckStatus[2] = {0};

  // !0 represent Chinese, 0 represent English
  unsigned char LanguageRecbuf;

  // PrinterStatusKey[1] value: 0 represents to keep temperature, 1 represents  to heating, 2 stands for cooling, 3 stands for printing, 4 pause for printing
  // PrinterStatusKey[0] value: 0 reprensents 3D printer ready
  char PrinterStatusKey[2] = {0};

  extern CardReader card;
  // represents SD-card status, true means SD is available, false means opposite.
  bool RTS_lcd_sd_status;

  char Checkfilenum = 0;
  int FilenamesCount = 0;
  char cmdbuf[20] = {0};
  char FilementStatus[2] = {0};

  // 0 for 10mm, 1 for 1mm, 2 for 0.1mm
  unsigned char AxisPagenum = 0;
  bool InforShowStatus = true;

  // true for only opening time and percentage, false for closing time and percentage.
  bool TPShowStatus = false;
  bool AutoLevelStatus = false;
  bool AutohomeKey = false;
  unsigned char AutoHomeIconNum;
  RTSSHOW rtscheck;
  int Update_Time_Value = 0;
  unsigned long VolumeSet = 0x80;

  bool PoweroffContinue = false;
  char commandbuf[30];

  unsigned char G29_status = 0;
  bool PreheatStatus[] = {false,false};

  bool flag_stop_change = false;
  bool flag_state_resumeprint = false;

  #define DEFAULT_LANGUAGE 0x00
  unsigned char language_change = DEFAULT_LANGUAGE;

  //语言更新
  void Set_Language(unsigned char num)
  {
    for(int Addr = PRINT_TITLE_CHAR_VP;Addr <= LEVEL_MODE_TITLE_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    rtscheck.RTS_SndData(num, PRINT_MAIN_CHAR_VP);
    rtscheck.RTS_SndData(num, TEMP_MAIN_CHAR_VP);
    rtscheck.RTS_SndData(num, SETTINGS_MAIN_CHAR_VP);
    delay(5);

    for(int Addr = BEGIN_PRINT_CHAR_VP;Addr <= STOP_PRINT_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }
    
    for(int Addr = PAUSE_PRINT_CHAR_VP;Addr <= ADJUST_PRINT_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = Z_OFFSET_Z_CHAR_VP;Addr <= BED_TEMP_Z_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = AUTO_TEMP_CHAR_VP;Addr <= FAN_TEMP_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = NOZZLE_PREHEAT_CHAR_VP;Addr <= COOLING_MANUAL_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = EXTRUDER_REFUL_CHAR_VP;Addr <= RETREAT2_REFUL_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = MACHINE_TYPE_ABOUT_CHAR_VP;Addr <= WEBSITE_ABOUT_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = LEVELING_SET_CHAR_VP;Addr <= ABOUT_SET_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = MEASURE_LEVEL_CHAR_VP;Addr <= AUTO_LEVEL_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = UNIT_MOVE_CHAR_VP;Addr <= Z_MOVE_CHAR_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = MOTOR_OFF_DIALOG_VP;Addr <= NO_2_DIALOG_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }

    for(int Addr = ASSIST_LEVEL_SIGNAL_VP;Addr <= PROCESSING_SIGNAL_VP;Addr++)
    {
        rtscheck.RTS_SndData(num, Addr);
        delay(2);
    }
    
    if(num == 1)
      rtscheck.RTS_SndData(CORP_WEBSITE_C, WEBSITE_TEXT_VP);
    else
      rtscheck.RTS_SndData(CORP_WEBSITE_E, WEBSITE_TEXT_VP);
    
  }

  inline void RTS_line_to_current(AxisEnum axis) 
  {
    if (!planner.is_full())
    {
      planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
    }
  }

  RTSSHOW::RTSSHOW()
  {
    recdat.head[0] = snddat.head[0] = FHONE;
    recdat.head[1] = snddat.head[1] = FHTWO;
    memset(databuf,0, sizeof(databuf));
  }

  void RTSSHOW::RTS_SDCardInit(void)
  {
    // if(!IS_SD_INSERTED())
    //if(!card.isMounted())
    {
    //  card.mount(); 
    }
    delay(2);
    // if (IS_SD_INSERTED())
    if(card.isMounted())
    {
      SERIAL_ECHOLN("***Initing card is OK***");
      uint16_t fileCnt = card.get_num_Files();
      card.getWorkDirName();
      if (card.filename[0] == '/') 
      {
        card.mount();
      }
      else 
      {
        card.cdup();
      }

      int addrnum =0;
      int num = 0;

      for (uint16_t i = 0; i < fileCnt && i < MaxFileNumber + addrnum; i++) 
      {
        card.selectFileByIndex(fileCnt-1-i);
        char *pointFilename = card.longFilename;
        int filenamelen = strlen(card.longFilename);
        int j = 1;
        while((strncmp(&pointFilename[j],".gcode",6) && strncmp(&pointFilename[j],".GCODE",6)) && (j++) < filenamelen);
        if(j >= filenamelen)
        {
          addrnum++;
          continue;
        }

        if(j >= TEXTBYTELEN)
        {
          strncpy(&card.longFilename[TEXTBYTELEN -3],"..",2);
          card.longFilename[TEXTBYTELEN-1] = '\0';
          j = TEXTBYTELEN-1;
        }

        delay(3);
        strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename,j);
        
        strcpy(CardRecbuf.Cardfilename[num], card.filename);
        //CardRecbuf.addr[num] = SDFILE_ADDR +num*10;
        CardRecbuf.addr[num] = FILE_FIRST_TEXT_VP + num * 20;
        RTS_SndData(CardRecbuf.Cardshowfilename[num],CardRecbuf.addr[num]);
        CardRecbuf.Filesum = (++num);

        //RTS_SndData(1, FilenameIcon + CardRecbuf.Filesum);
      }

      //RTS_SndData(0,IconPrintstatus);  //显示准备就绪
      RTS_SndData(language_change +63 -1,STATUS_DP_CHAR_VP);
      RTS_SndData(language_change +63 -1,STATUS_DP_CHAR_VP); //显示准备就绪
      msgStatus = MSG_READY;
      for(int j = 0;j < CardRecbuf.Filesum;j++)
      {
        RTS_SndData(0,FILE_SELECT_1_ICON_VP+j);
      }

      // RTS_lcd_sd_status = IS_SD_INSERTED();
      RTS_lcd_sd_status = card.isMounted();
    }
    else
    {
      SERIAL_ECHOLN("***Initing card fails***");

      //RTS_SndData(6, IconPrintstatus); //卡已移除 待定
      RTS_SndData(language_change +54 -1,STATUS_DP_CHAR_VP);
      msgStatus = MSG_CARD_PULLOUT;
    }
  }

  void RTSSHOW::RTS_SDCardUpate(void)
  {
    const bool sd_status = IS_SD_INSERTED();

    if (sd_status != RTS_lcd_sd_status)
    {
      if (sd_status)
      {
        delay(500);
        card.mount();     //挂载SD卡
        RTS_SDCardInit(); //初始化SD卡
      }
      else
      {
        card.release();
        // heating or printing
        if(CardCheckStatus[0] == 1)
        {
          card.flag.abort_sd_printing = 1;
          wait_for_heatup = false;

          CardCheckStatus[0] = 0;  // cancel to check card during printing the gcode file 

          // back main page
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }

        //RTS_SndData(6,IconPrintstatus); //卡已移除 待定

        //卡已移除
        //RTS_SndData(6,IconPrintstatus);
        RTS_SndData(language_change +54 -1,STATUS_DP_CHAR_VP);
        msgStatus = MSG_CARD_PULLOUT;
        for(int i = 0;i < CardRecbuf.Filesum;i++)
        {
          for(int j = 0;j < 10;j++)
          RTS_SndData(0, CardRecbuf.addr[i] + j);
          // RTS_SndData(4,FilenameIcon + 1 + i);
          // white
          //RTS_SndData((unsigned long)0xFFFF,FilenameNature + (i + 1) * 16);
        }

        /*
        for(int j = 0;j < 10;j++)
        {
          RTS_SndData(0, Printfilename + j);  // clean screen.
          RTS_SndData(0, Choosefilename + j); // clean filename
        }
        for(int j = 0;j < 8;j ++)
        {
          RTS_SndData(0, FilenameCount + j);
        }
        // clean filename Icon
        for(int j = 1;j <= 20;j++)
        {
          RTS_SndData(10, FilenameIcon+j);
          RTS_SndData(10, FilenameIcon1+j);
        }
        */

        //清除
        for(int i = 0;i < CardRecbuf.Filesum;i++)
        {
          for(int j = 0;j < 10;j++)
          RTS_SndData(0,CardRecbuf.addr[i]+j);
        }

        for(int j = 0;j < 10;j++)
        {
          RTS_SndData(0,FILE_SELECTED_TEXT_VP+j); // clean screen.
        }

        // clean filename Icon
        for(int j = 0;j < 20;j++)
        {
          RTS_SndData(10,FILE_SELECT_1_ICON_VP+j);
        }

        memset(&CardRecbuf,0,sizeof(CardRecbuf));
      }
      RTS_lcd_sd_status = sd_status;
    }
    //RTS_lcd_sd_status = sd_status;

    // represents to update file list
    // if (CardUpdate && RTS_lcd_sd_status && IS_SD_INSERTED())
    // {
    //   // clean filename
    //   for(int j = 0;j < 10;j++)
    //   {
    //     RTS_SndData(0,Choosefilename+j);
    //   }
    //   for(int j = 0;j < 8;j++)
    //   {
    //     RTS_SndData(0,FilenameCount+j);
    //   }
    //   for (uint16_t i = 0; i < CardRecbuf.Filesum ; i++) 
    //   {
    //     delay(3);
    //     RTS_SndData(CardRecbuf.Cardshowfilename[i],CardRecbuf.addr[i]);
    //     RTS_SndData(1,FilenameIcon+1+i);
    //     RTS_SndData((unsigned long)0xFFFF,FilenameNature + (i+1)*16);   // white
    //     RTS_SndData(10,FilenameIcon1+1+i);
    //   }
    //   CardUpdate = false;
    // }

    // represents to update file list
    if (CardUpdate && RTS_lcd_sd_status && IS_SD_INSERTED())
    {
      for (uint16_t i = 0; i < CardRecbuf.Filesum ; i++) 
      {
        delay(3);
        RTS_SndData(CardRecbuf.Cardshowfilename[i],CardRecbuf.addr[i]);
       // RTS_SndData(1,FilenameIcon+1+i);
        //RTS_SndData((unsigned long)0xFFFF,FilenameNature + (i+1)*16);   // white
        RTS_SndData(0,FILE_SELECT_1_ICON_VP+i);
      }
      CardUpdate = false;
    }
  }

  int RTSSHOW::RTS_CheckFilement(int mode)
  {
    waitway = 4;

    // no filements check
    for(Checkfilenum = 0; (1 == READ(CHECKFILEMENT_PIN)) && (Checkfilenum < 50); Checkfilenum++)
    {
      delay(15);
    }

    if(Checkfilenum>=49)
    {
      // no filements
      if(mode)
      {
        // for mode status of no filement . the sentence can be canceled, which isn't neccessary?
        FilementStatus[0] = mode;
        RTS_SndData(ExchangePageBase + 38, ExchangepageAddr);
      }
      Checkfilenum = 0;
      waitway = 0;
      return 1;
    }
    else
    {
      Checkfilenum = 0;
    }

    waitway = 0;
    return 0;
  }

  void RTSSHOW::RTS_Init()
  {
    BL24CXX_Read(FONT_EEPROM, (uint8_t*)&language_change, sizeof(language_change));

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      BL24CXX_Read(FONT_EEPROM+2, (uint8_t*)&AutoLevelStatus, sizeof(AutoLevelStatus));

      if(AutoLevelStatus) 
      {
        RTS_SndData(2, AUTOLEVEL_SWITCH_ICON_VP);
        // settings.reset();
        queue.enqueue_now_P(PSTR("M420 S0"));
      }
      else 
      {
        bool zig = true;
        int8_t inStart, inStop, inInc, showcount;
        showcount = 0;
        RTS_SndData(3, AUTOLEVEL_SWITCH_ICON_VP);
        settings.load();
        for(int y = 0;y < GRID_MAX_POINTS_Y;y++)
        {
          // away from origin
          if (zig) 
          {
            inStart = 0;
            inStop = GRID_MAX_POINTS_X;
            inInc = 1;
          }
          else
          {
            // towards origin
            inStart = GRID_MAX_POINTS_X - 1;
            inStop = -1;
            inInc = -1;
          }
          zig ^= true;
          for(int x = inStart;x != inStop; x += inInc)
          {
            RTS_SndData(z_values[x][y]*1000, AutolevelVal + showcount*2);
            showcount++;
          }
        }
        queue.enqueue_now_P(PSTR("M420 S1"));
      }
    #endif

    last_zoffset = zprobe_zoffset;
    RTS_SndData(zprobe_zoffset*100, 0x1026);
    
    last_target_temperature_bed = thermalManager.temp_bed.target;
    last_target_temperature[0] =  thermalManager.temp_hotend[0].target;
    feedrate_percentage = 100;
    //RTS_SndData(feedrate_percentage, FeedrateDisplay);
    RTS_SndData(feedrate_percentage, PRINT_PERCENT_DATA_VP);

    RTS_SndData(0, MOTOR_STATUS_ICON_VP); //显示关闭电机使能
    Motor_Switch = 0;

    /***************transmit temperature to screen*****************/
    RTS_SndData(0, NOZZLE_PREHEAT_DATA_VP);
    RTS_SndData(0, BED_PREHEAT_DATA_VP);
    RTS_SndData(thermalManager.temp_hotend[0].celsius, NOZZLE_TEMP_DATA_VP);
    RTS_SndData(thermalManager.temp_bed.celsius, BED_TEMP_DATA_VP);

    /***************transmit Fan speed to screen*****************/
    // 注意
    // turn off fans
    thermalManager.set_fan_speed(0, FanOff);
    // RTS_SndData(2, FanKeyIcon);
    // RTS_SndData(2, LCDKeyIcon);
    RTS_SndData(0, FAN_SWITCH_ICON_VP);
    
    /***************transmit Printer information to screen*****************/
    // clean filename
    // for(int j = 0;j < 20;j++)
    // {
    //   RTS_SndData(0,MacVersion+j);
    // }

    //char sizebuf[20]={0};
    //sprintf(sizebuf,"%d X %d X %d", MAC_LENGTH, MAC_WIDTH, MAC_HEIGHT);
    //RTS_SndData(MACVERSION, MacVersion);
    //RTS_SndData(SOFTVERSION, SoftVersion);
    //RTS_SndData(sizebuf, PrinterSize);
    RTS_SndData(SOFTVERSION, FW_VERSION_TEXT_VP);
    //RTS_SndData(sizebuf, PRINT_SIZE_TEXT_VP);

    if(LanguageRecbuf != 0)
      RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_CHAR_VP);
    else
      RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_CHAR_VP);

    /**************************some info init*******************************/
    //RTS_SndData(0,PrintscheduleIcon);
    //RTS_SndData(0,PrintscheduleIcon+1);

    /************************clean screen*******************************/
    for(int i = 0;i < MaxFileNumber;i++)
    {
      //  RTS_SndData(10,FILE_SELECT_1_ICON_VP+i);
        for(int j = 0;j < 10;j++)
        {
            RTS_SndData(0,FILE_FIRST_TEXT_VP +i*20+j);
        }
    }

    // clean filename Icon
    for(int j = 0;j < 20;j++)
    {
      RTS_SndData(10,FILE_SELECT_1_ICON_VP+j);
    }
    memset(&CardRecbuf,0,sizeof(CardRecbuf));

    /*********transmit SD card filename to screen***************/
    RTS_SDCardInit();

    rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);           // Boot progress bar
    for(startprogress = 0; startprogress <= 100; startprogress++)
    {
        rtscheck.RTS_SndData(startprogress,START_PROCESS_ICON_VP);
        delay(12);
    }

    if(language_change < 0x01 || language_change > 0x09)
    {
      rtscheck.RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
    }
    else
    {
      Set_Language(language_change);
      rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
    }
    rtscheck.RTS_SndData(0,START_PROCESS_ICON_VP);

    SERIAL_ECHOLN("\n===Initing RTS has finished===");
  }

  int RTSSHOW::RTS_RecData()
  {
    while(MYSERIAL1.available() > 0 && (recnum < SizeofDatabuf))
    {
      delay(1);
      databuf[recnum] = MYSERIAL1.read();
      if(databuf[0] == FHONE)
      {
        recnum++;
      }
      else if(databuf[0] == FHTWO)
      {
        databuf[0] = FHONE;
        databuf[1] = FHTWO;
        recnum += 2;
      }
      else if(databuf[0] == FHLENG)
      {
        databuf[0] = FHONE;
        databuf[1] = FHTWO;
        databuf[2] = FHLENG;
        recnum += 3;
      }
      else if(databuf[0] == VarAddr_R)
      {
        databuf[0] = FHONE;
        databuf[1] = FHTWO;
        databuf[2] = FHLENG;
        databuf[3] = VarAddr_R;
        recnum += 4;
      }
      else
      {
        recnum = 0;
      }
    }

    // receive nothing
    if(recnum < 1)
    {
      return -1;
    }
    else  if((recdat.head[0] == databuf[0]) && (recdat.head[1] == databuf[1]) && recnum > 2)
    {
      recdat.len = databuf[2];
      recdat.command = databuf[3];
      if(recdat.len == 0x03 && (recdat.command == 0x82 || recdat.command == 0x80) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))  //response for writing byte
      {   
        memset(databuf,0, sizeof(databuf));
        recnum = 0;
        return -1;
      }
      else if(recdat.command == 0x83)
      {
        // response for reading the data from the variate
        recdat.addr = databuf[4];
        recdat.addr = (recdat.addr << 8 ) | databuf[5];
        recdat.bytelen = databuf[6];
        for(unsigned int i = 0;i < recdat.bytelen;i+=2)
        {
          recdat.data[i/2]= databuf[7+i];
          recdat.data[i/2]= (recdat.data[i/2] << 8 )| databuf[8+i];
        }
      }
      else if(recdat.command == 0x81)
      {
        // response for reading the page from the register
        recdat.addr = databuf[4];
        recdat.bytelen = databuf[5];
        for(unsigned int i = 0;i < recdat.bytelen;i++)
        {
          recdat.data[i]= databuf[6+i];
          // recdat.data[i]= (recdat.data[i] << 8 )| databuf[7+i];
        }
      }
    }
    else
    {
      memset(databuf,0, sizeof(databuf));
      recnum = 0;
      // receive the wrong data
      return -1;
    }
    memset(databuf,0, sizeof(databuf));
    recnum = 0;
    return 2;
  }

  void RTSSHOW::RTS_SndData(void)
  {
    if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && snddat.len >= 3)
    {
      databuf[0] = snddat.head[0];
      databuf[1] = snddat.head[1];
      databuf[2] = snddat.len;
      databuf[3] = snddat.command;
      // to write data to the register
      if(snddat.command ==0x80)
      {
        databuf[4] = snddat.addr;
        for(int i =0;i <(snddat.len - 2);i++)
        {
          databuf[5+i] = snddat.data[i];
        }
      }
      else if(snddat.len == 3 && (snddat.command ==0x81))
      {
        // to read data from the register
        databuf[4] = snddat.addr;
        databuf[5] = snddat.bytelen;
      }
      else if(snddat.command ==0x82)
      {
        // to write data to the variate
        databuf[4] = snddat.addr >> 8;
        databuf[5] = snddat.addr & 0xFF;
        for(int i =0;i <(snddat.len - 3);i += 2)
        {
          databuf[6 + i] = snddat.data[i/2] >> 8;
          databuf[7 + i] = snddat.data[i/2] & 0xFF;
        }
      }
      else if(snddat.len == 4 && (snddat.command ==0x83))
      {
        // to read data from the variate
        databuf[4] = snddat.addr >> 8;
        databuf[5] = snddat.addr & 0xFF;
        databuf[6] = snddat.bytelen;
      }
      usart_tx(MYSERIAL1.c_dev(), databuf, snddat.len + 3);
      MYSERIAL1.flush();
      // for(int i = 0;i < (snddat.len + 3);i++)
      // {
      //   MYSERIAL1.write(databuf[i]);
      //   delayMicroseconds(1);
      // }

      memset(&snddat,0,sizeof(snddat));
      memset(databuf,0, sizeof(databuf));
      snddat.head[0] = FHONE;
      snddat.head[1] = FHTWO;
    }
  }

  void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
  {
    if(s.length() < 1)
    {
      return;
    }
    RTS_SndData(s.c_str(), addr, cmd);
  }

  void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
  {
    int len = strlen(str);
    if( len > 0)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = 3+len;
      databuf[3] = cmd;
      databuf[4] = addr >> 8;
      databuf[5] = addr & 0x00FF;
      for(int i =0;i <len ;i++)
      {
        databuf[6 + i] = str[i];
      }

      for(int i = 0;i < (len + 6);i++)
      {
        MYSERIAL1.write(databuf[i]);
        delayMicroseconds(1);
      }
      memset(databuf,0, sizeof(databuf));
    }
  }

  void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
  {
    snddat.command = cmd;
    snddat.addr = addr;
    snddat.data[0] = (unsigned long)c;
    snddat.data[0] = snddat.data[0] << 8;
    snddat.len = 5;
    RTS_SndData();
  }

  void RTSSHOW::RTS_SndData(unsigned char* str, unsigned long addr, unsigned char cmd){RTS_SndData((char *)str, addr, cmd);}

  void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
  {
    if(cmd == VarAddr_W )
    {
      if(n > 0xFFFF)
      {
        snddat.data[0] = n >> 16;
        snddat.data[1] = n & 0xFFFF;
        snddat.len = 7;
      }
      else
      {
        snddat.data[0] = n;
        snddat.len = 5;
      }
    }
    else if(cmd == RegAddr_W)
    {
      snddat.data[0] = n;
      snddat.len = 3;
    }
    else if(cmd == VarAddr_R)
    {
      snddat.bytelen = n;
      snddat.len = 4;
    }
    snddat.command = cmd;
    snddat.addr = addr;
    RTS_SndData();
  }

  void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd){ RTS_SndData((int)n, addr, cmd); }

  void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd){ RTS_SndData((int)n, addr, cmd); }

  void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd){ RTS_SndData((unsigned long)n, addr, cmd); }

  void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
  {
    if(cmd == VarAddr_W )
    {
      if(n > 0xFFFF)
      {
        snddat.data[0] = n >> 16;
        snddat.data[1] = n & 0xFFFF;
        snddat.len = 7;
      }
      else
      {
        snddat.data[0] = n;
        snddat.len = 5;
      }
    }
    else if(cmd == VarAddr_R)
    {
      snddat.bytelen = n;
      snddat.len = 4;
    }
    snddat.command = cmd;
    snddat.addr = addr;
    RTS_SndData();
  }

  void RTSSHOW::RTS_SDcard_Stop()
  {
    waitway = 7;
    if(home_flag) planner.synchronize();
    if(heat_flag)
    {
      card.stopSDPrint();
      queue.clear();
      quickstop_stepper();
      print_job_timer.stop();
      #if DISABLED(SD_ABORT_NO_COOLDOWN)
        thermalManager.disable_all_heaters();
      #endif
      thermalManager.zero_fan_speeds();
      wait_for_heatup = false;
      #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
        card.removeJobRecoveryFile();
      #endif
      #ifdef EVENT_GCODE_SD_STOP
        queue.inject_P(PSTR(EVENT_GCODE_SD_STOP));        
      #endif
    }
    else
    {
      card.flag.abort_sd_printing = true;
    }
    
    PrinterStatusKey[0] = 0;
    G29_status = 0;
    AutohomeKey = false;

    // shut down the stepper motor.
    // queue.enqueue_now_P(PSTR("M84"));
    //RTS_SndData(11, FilenameIcon); 
    RTS_SndData(0, MOTOR_STATUS_ICON_VP); 
    Motor_Switch = 0;
    
    // for system
    // delay(1000);
    // RTS_SndData(0,Timehour);
    // delay(2);
    // RTS_SndData(0,Timemin);
    // delay(2);

    // RTS_SndData(0,PrintscheduleIcon);
    // RTS_SndData(0,PrintscheduleIcon+1);
    // RTS_SndData(0,Percentage);

    RTS_SndData(0,START_PROCESS_ICON_VP);
    RTS_SndData(0,PRINT_PERCENT_DATA_VP);

    delay(2);

/*     for(int j = 0;j < 10;j++)
    {
      // clean screen.
      RTS_SndData(0,Printfilename+j);
    // clean filename
      RTS_SndData(0,Choosefilename+j);
    } */

    for(int j = 0;j < 10;j++)
    {
      // clean screen.
      RTS_SndData(0,FILE_SELECTED_TEXT_VP+j);
    // clean filename
    //  RTS_SndData(0,Choosefilename+j);
    }

    // for(int j = 0;j < 8;j++)
    // {
    //   RTS_SndData(0,FilenameCount+j);
    // }

    InforShowStatus = true;
    TPShowStatus = false;
  }

  void RTSSHOW::RTS_HandleData()
  {
    AxisEnum axis;
    float min, max;

    // for waiting
    if(waitway > 0)
    {
      memset(&recdat,0 , sizeof(recdat));
      recdat.head[0] = FHONE;
      recdat.head[1] = FHTWO;
      return;
    }

    if(recdat.addr < 0)
    {
      memset(&recdat,0 , sizeof(recdat));
      recdat.head[0] = FHONE;
      recdat.head[1] = FHTWO;
      return;
    }

    // if(recdat.addr != 0x00 && recdat.addr != Root_Key)
    // {
    //   Root_Flag = 0;
    // }

    switch(recdat.addr)
    {
      //case Printfile:
      case MainPage:  //0x2001
      {
        if(recdat.data[0] == 1)   //点击"打印"按钮
        {
          InforShowStatus = false;
          CardUpdate = true;
          CardRecbuf.recordcount = -1;
          RTS_SDCardUpate();
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr); // exchange to 2 page
        }
        else if(recdat.data[0] == 2)   //点击"打印完成"按钮
        {
          //切换打印首界面

          // exchange to 1 page
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);

          thermalManager.disable_all_heaters(); // 'unpause' taken care of in here
          print_job_timer.stop();  
             
          // RTS_SndData(0,Timehour);
          // RTS_SndData(0,Timemin);
          RTS_SndData(0,PRINT_TIMEHOUR_DATA_VP);
          RTS_SndData(0,PRINT_TIMEMIN_DATA_VP);

          Update_Time_Value = 0;

          // close the key of checking card in printing
          CardCheckStatus[0] = 0;
          tempbed = 0;
          temphot = 0;
                    
          InforShowStatus = true;
          TPShowStatus = false;
          quickstop_stepper(); 
   
          // RTS_SndData(11, FilenameIcon); 
          // RTS_SndData(0,PrintscheduleIcon);
          // RTS_SndData(0,PrintscheduleIcon+1);
          // RTS_SndData(0,Percentage);
          // //delay(2);
          // RTS_SndData(0,Timehour);
          // RTS_SndData(0,Timemin);

          RTS_SndData(0,MOTOR_STATUS_ICON_VP);
          Motor_Switch = 0;

          RTS_SndData(0,START_PROCESS_ICON_VP);
          RTS_SndData(0,PRINT_PERCENT_DATA_VP);
          delay(2);
          RTS_SndData(0,PRINT_TIMEHOUR_DATA_VP);
          RTS_SndData(0,PRINT_TIMEMIN_DATA_VP);

          print_job_timer.reset();
          
          //RTS_SndData(10, FilenameIcon); 
        } 
        else if(recdat.data[0] == 3)   // 点击温度按钮
        {
          InforShowStatus = true;
          TPShowStatus    = false;

          if(thermalManager.fan_speed[0])
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr); //开风扇界面
          }
          else
          {
            RTS_SndData(ExchangePageBase + 13, ExchangepageAddr); //关风扇界面
          }
        }
        else if(recdat.data[0] == 4 )   // Settings
        {
          RTS_SndData(ExchangePageBase + 17, ExchangepageAddr); //开风扇界面
          InforShowStatus = false;
        }
      }
      break;

      //case Ajust: 
      case PrintOptions: //打印界面下点击“调整”按钮 0x2007
      {
        if(recdat.data[0] == 0x01) //打印界面点击停止打印
        {
          RTS_SndData(ExchangePageBase + 31, ExchangepageAddr); 
        }
        else if(recdat.data[0] == 0x02) //点击“调整”  
        {
          // InforShowStatus = false;
          thermalManager.fan_speed[0] ? RTS_SndData(1, FAN_SWITCH_ICON_VP):RTS_SndData(0, FAN_SWITCH_ICON_VP);
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr); //切换调整页面
        }
        else if(recdat.data[0] == 0x03) 
        {
          RTS_SndData(ExchangePageBase + 32, ExchangepageAddr); 
        }
        else if(recdat.data[0] == 0x11) //调整界面，风扇调整
        {
          // turn on the fan
          if(thermalManager.fan_speed[0])
          {
             RTS_SndData(0, FAN_SWITCH_ICON_VP); 
            thermalManager.set_fan_speed(0, FanOff);
          }
          else
          {
            // turn off the fan
            RTS_SndData(1, FAN_SWITCH_ICON_VP);
            thermalManager.set_fan_speed(0, FanOn);
          }
        }
        else if(recdat.data[0] == 0x04)
        {

        }
        else if(recdat.data[0] == 0x05) //在打印界面点击“继续”按钮
        {
          //置处于继续打印状态标志位
          flag_state_resumeprint = true;

          FilementStatus[0] = 2;          //add 2021-03-25

          #if CHECKFILEMENT
            /**************checking filement status during printing************/
            if(RTS_CheckFilement(0)) 
            {
              if(thermalManager.temp_hotend[0].target>= 185)
              {
                for(startprogress = 0;startprogress < 5;startprogress++)
                {
                  RTS_SndData(startprogress, REFUEL_GIF_FILEMENT_VP);
                  delay(400);
                }
              }
              else
              {
                rtscheck.RTS_SndData(ExchangePageBase + 38, ExchangepageAddr);
              }
              break;
            }
          #endif
          
          //切换为无暂停按钮的正在预热界面
          RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);

          PrinterStatusKey[1] = 3; 
          //PrintStatue[1] = 2; 

          char pause_str_Z[16];
          char pause_str_E[16];
          memset(pause_str_Z, 0, sizeof(pause_str_Z));
          dtostrf(pause_z, 3, 2, pause_str_Z);
          memset(pause_str_E, 0, sizeof(pause_str_E));
          dtostrf(pause_e, 3, 2, pause_str_E);

          if(!PrintMode)
          {
            memset(commandbuf,0,sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("M190 S%i"), tempbed);
            gcode.process_subcommands_now(commandbuf);
          }
          
          memset(commandbuf,0,sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
          gcode.process_subcommands_now(commandbuf);
          memset(commandbuf,0,sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("G0 Z%s"), pause_str_Z);
          gcode.process_subcommands_now(commandbuf);
          sprintf_P(commandbuf, PSTR("G92 E%s"), pause_str_E);
          gcode.process_subcommands_now(commandbuf);
          tempbed = 0;
          temphot = 0;

          card.startFileprint();
          print_job_timer.start();

          //切换为正在打印的标题
          if(!flag_stop_change) //用于解决在继续打印过程中点击停止按钮会出现11页面的情况
          {
            //RTS_SndData(2,IconPrintstatus);  //待定  
            RTS_SndData(language_change +72 -1,STATUS_DP_CHAR_VP);// for Heating
            msgStatus = MSG_HEATING;
          }

          PrintStatue[1]      = 0; //PrintStatue[1]=0 表示为发生了继续打印事件标志位 

          Update_Time_Value   = 0;

          //PrinterStatusKey[1] = 3; //PrinterStatusKey[1]=3 表示为stands for printing
          PrinterStatusKey[1] = 0; //PrinterStatusKey[1]=3 表示为stands for printing

          // open the key of checking card in printing
          CardCheckStatus[0] = 1;

          if(!flag_stop_change) //用于解决在继续打印过程中点击停止按钮会出现11页面的情况
          {
            RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
          }
          flag_stop_change = false;
          flag_state_resumeprint = false;                      
        }
        // else if(recdat.data[0] == 0x06) //在断料提示界面，点击“是”
        // {
        //   NozzleTempStatus[2] = 1;
        //   thermalManager.temp_hotend[0].target = temphot;
        //   startprogress  = 0;
        //   RTS_SndData(ExchangePageBase + 43, ExchangepageAddr);
        // }
        else if(recdat.data[0] == 0x10) //点击返回键
        {
          InforShowStatus = true;
          
          if(PrinterStatusKey[1] == 3) // 预热过程中的返回页面 during heating
          {
            // in heating
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
          }
          else if(PrinterStatusKey[1] == 4) //暂停 → 调整 → 暂停
          {
            // in the pause
            RTS_SndData(ExchangePageBase + 9, ExchangepageAddr);
          }
          else
          {
            // in printing
            RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
          }
        }
        else if(recdat.data[0] == 0xF3) //停止打印按“否”
        {
          if(PrintStatue[1] == 1)
          {
            if(PrinterStatusKey[1] == 3)
            {
              RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            }
            else
            {
              RTS_SndData(ExchangePageBase + 9, ExchangepageAddr);
            }
          }
          else if(PrintStatue[1] == 0)
          {
            // during heating
            if(PrinterStatusKey[1] == 3)
            {
              // in heating
              RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            }
            else
            {
              // in printing
              RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
            }
          }
        }
        else if(recdat.data[0] == 0xF1) //暂停打印弹窗按“否”
        {
          RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
        }
        else if(recdat.data[0] == 0xF4) //停止打印按“是”
        {
          // recover the status waiting to check filements
          FilementStatus[0]  =  0;

          state_stop_flag = 1;

          //切换“正在操作中请稍后”界面
          RTS_SndData(ExchangePageBase + 43, ExchangepageAddr);

          thermalManager.disable_all_heaters(); // 'unpause' taken care of in here
          print_job_timer.stop();  
            
          RTS_SndData(0,PRINT_TIMEHOUR_DATA_VP);
          RTS_SndData(0,PRINT_TIMEMIN_DATA_VP);

          Update_Time_Value = 0;

          // close the key of checking card in printing
          CardCheckStatus[0] = 0;
          tempbed = 0;
          temphot = 0;

          RTS_SDcard_Stop();
          //RTS_SndData(10, FilenameIcon);

          if(flag_state_resumeprint) //如果前一状态处于继续打印状态
          {
            flag_stop_change = true;
          }
        }
        else if(recdat.data[0] == 0xF2) //暂停打印弹窗按“是”
        {
          RTS_SndData(ExchangePageBase + 43, ExchangepageAddr); //切换正在进行中

          // reject to receive cmd
          waitway = 1;
          pause_z = current_position[Z_AXIS];
          pause_e = current_position[E_AXIS] - 2;

          card.pauseSDPrint();
          print_job_timer.pause();

          temphot = thermalManager.degTargetHotend(0);
          tempbed = thermalManager.degTargetBed();

          thermalManager.setTargetHotend(0, 0);
          //thermalManager.setTargetBed(0);

          // for return the corresponding page
          // 置相应标志位
          PrintStatue[1]      = 1;      //PrintStatue[1]==1 表示当前页面为暂停页面
          pause_action_flag   = true;
          PrinterStatusKey[1] = 4;      //用于单击返回按键可以返回正确的页面

          Update_Time_Value = 0;
        }
      }
      break;

      //case Feedrate:
      case PRINT_SPEED_KEY: //0x1414
      {
        feedrate_percentage = recdat.data[0];
        RTS_SndData(feedrate_percentage, PRINT_SPEED_DATA_VP);
      }
      break;

      //case Zoffset:
      case Z_OFFSET_DATA_VP:
      {
        last_zoffset = zprobe_zoffset;
        if(recdat.data[0] >= 32768)
        {
          zprobe_zoffset = ((float)recdat.data[0]-65536)/100;
        }
        else
        {
          zprobe_zoffset = ((float)recdat.data[0])/100;
        }
        if (WITHIN((zprobe_zoffset), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
        {
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
        }
      }
      break;

      //case TempControl:
      case TempMenu:  //点击温度选项卡进入该处理段 0x200C
      {
        if(recdat.data[0] == 0x00) //点击"返回"键
        {
          InforShowStatus = true;
          TPShowStatus = false;
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }
        else if(recdat.data[0] == 0x01) //点击“自动设温”
        {

        }
        else if(recdat.data[0] == 0x02) //点击“手动设温”
        {

        }
        else if(recdat.data[0] == 0x03) //点击“风扇”图标
        {
          if(thermalManager.fan_speed[0]) //关风扇
          {
            thermalManager.fan_speed[0] = FanOff;
            RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
          }
          else //开风扇
          {
            thermalManager.fan_speed[0] = FanOn;
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
        }
        else if(recdat.data[0] == 0x05)   //点击"PLA模式"按钮
        {
          thermalManager.setTargetHotend((PLA_ABSModeTemp = PREHEAT_1_TEMP_HOTEND), 0);
          thermalManager.setTargetBed(PREHEAT_1_TEMP_BED);
          RTS_SndData(PREHEAT_1_TEMP_HOTEND,NOZZLE_PREHEAT_DATA_VP);
          RTS_SndData(PREHEAT_1_TEMP_BED,BED_PREHEAT_DATA_VP);
          /* 退出模式界面 */
          if(thermalManager.fan_speed[0]) //在风扇开启的状态
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
          else //在风扇关闭的状态
          {
            RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
          }
        }
        else if(recdat.data[0] == 0x06)   //点击“ABS模式”按钮
        {
          thermalManager.setTargetHotend((PLA_ABSModeTemp = PREHEAT_2_TEMP_HOTEND), 0);
          thermalManager.setTargetBed(PREHEAT_2_TEMP_BED);
          RTS_SndData(PREHEAT_2_TEMP_HOTEND,NOZZLE_PREHEAT_DATA_VP);
          RTS_SndData(PREHEAT_2_TEMP_BED,BED_PREHEAT_DATA_VP);
          /* 退出模式界面 */
          if(thermalManager.fan_speed[0]) //在风扇开启的状态
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
          else //在风扇关闭的状态
          {
            RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
          }
        }
        else if(recdat.data[0] == 0xF1) //冷却硬切换后，进入弹窗界面，点击“是”进行冷却
        {
          if(thermalManager.fan_speed[0]) //在风扇开启的状态
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
          else //在风扇关闭的状态
          {
            RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
          }
        }
        else if(recdat.data[0] == 0xF2)
        {
          // InforShowStatus = true;
          #if FAN_COUNT > 0
            for (uint8_t i = 0; i < FAN_COUNT; i++) thermalManager.fan_speed[i] = FanOn;
          #endif
          thermalManager.disable_all_heaters();

          RTS_SndData(0,NOZZLE_PREHEAT_DATA_VP);
          delay(1);
          RTS_SndData(0,BED_PREHEAT_DATA_VP);
          delay(1);

          //RTS_SndData(8,IconPrintstatus); //正在冷却
          RTS_SndData(language_change +108 -1,STATUS_DP_CHAR_VP);
          msgStatus = MSG_COOLING;
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);

          PrinterStatusKey[1] = 2;          
        }
      }
      break;

      case AutoTemp:  //0x200E
      {
        if(recdat.data[0] == 0x01)
        {
          if(thermalManager.fan_speed[0]) //在风扇开启的状态
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
          else //在风扇关闭的状态
          {
            RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
          }
        }
      }
      break;
      
      case NOZZLE_PREHEAT_DATA_VP: //手动设温，输入喷头预热数值
      {
        thermalManager.temp_hotend[0].target = recdat.data[0];
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, NOZZLE_PREHEAT_DATA_VP); 
      }
      break;
            
      case BED_PREHEAT_DATA_VP: //手动设温，输入热床预热数值
      {
        thermalManager.temp_bed.target = recdat.data[0];
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_PREHEAT_DATA_VP);
      } 
      break;

      //case ManualSetTemp:
      case ManualTemp:      //手动设温
      {
        if(recdat.data[0] == 0x01)    //返回键
        {
          thermalManager.fan_speed[0] ? RTS_SndData(ExchangePageBase + 12, ExchangepageAddr):RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
        }
        else if(recdat.data[0] == 3)  //喷头一键降温
        {
          thermalManager.setTargetHotend(0, 0);
          RTS_SndData(0,NOZZLE_PREHEAT_DATA_VP);
          PrinterStatusKey[1] = 0;
        }
        else if(recdat.data[0] == 5)  //热床一键降温
        {
          thermalManager.setTargetBed(0);
          RTS_SndData(0,BED_PREHEAT_DATA_VP);
          PrinterStatusKey[1] = 0;
        }
      }
      break;

      //case Setting:
      case SettingsMenu:
      {
        if(recdat.data[0] == 1)   //点击返回键，返回首页
        {
          InforShowStatus = true;
          TPShowStatus = false;
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }
        else if(recdat.data[0] == 2)   //点击"调平"按钮
        {
          //切换电机图标
          RTS_SndData(0, MOTOR_STATUS_ICON_VP);
          Motor_Switch = 0;

          // only for prohibiting to receive massage
          waitway = 6;
          InforShowStatus = AutohomeKey = true;
          AutoHomeIconNum = 0;
          Update_Time_Value = 0;
          
          queue.enqueue_now_P(PSTR("G28"));
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));

          RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
          RTS_SndData(1, HOME_AUTORUN_VP);delay(2);
          AutoHomeIconNum = 1;
        }
        else if(recdat.data[0] == 3) //点击"换料"按钮
        {
          InforShowStatus = true;
          TPShowStatus = false;
          memset(ChangeMaterialbuf,0,sizeof(ChangeMaterialbuf));
          ChangeMaterialbuf[1]=ChangeMaterialbuf[0] = 10; 
          RTS_SndData(10*ChangeMaterialbuf[0], CHANGE_FILAMENT_DATA_VP); // It's ChangeMaterialbuf for show,instead of current_position[E_AXIS] in them.
          //RTS_SndData(10*ChangeMaterialbuf[1], FilementUnit2);
          RTS_SndData(thermalManager.temp_hotend[0].celsius,NOZZLE_TEMP_DATA_VP);
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target,NOZZLE_PREHEAT_DATA_VP);
          delay(2);
          RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        }
        else if(recdat.data[0] == 4)   //点击"移动轴"
        {
          // InforShowoStatus = false;
          AxisPagenum = 4;
          RTS_SndData(10*current_position[X_AXIS], X_MOVE_DATA_KEY); 
          RTS_SndData(10*current_position[Y_AXIS], Y_MOVE_DATA_KEY);
          rtscheck.RTS_SndData(10*current_position[Z_AXIS], Z_MOVE_DATA_KEY);
          delay(2);

          // Move axis, 10mm
          RTS_SndData(ExchangePageBase + 27, ExchangepageAddr);
        }
        else if(recdat.data[0] == 5)   //点击"电机"按钮
        {
          if(Motor_Switch)
          {
              queue.enqueue_now_P(PSTR("M84"));
              RTS_SndData(0, MOTOR_STATUS_ICON_VP);
              Motor_Switch =0 ;
          }
          else
          {
              queue.enqueue_now_P(PSTR("M17"));
              RTS_SndData(1, MOTOR_STATUS_ICON_VP);
              Motor_Switch =1 ;
          }
        }
        else if(recdat.data[0] == 6)   //点击"语言"按钮
        {
          RTS_SndData(ExchangePageBase + 29, ExchangepageAddr);
        }
        else if(recdat.data[0] == 7)   //点击"关于"按钮
        {
          /***************  information  ****************/
          RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_TEXT_VP);
          RTS_SndData(HARDWARE_VERSION, HW_VERSION_TEXT_VP);
          RTS_SndData(FIRMWARE_VERSION, FW_VERSION_TEXT_VP);
          // RTS_SndData(SCREEN_VERSION, DP_VERSION_TEXT_VP);
          RTS_SndData(PRINT_SIZE, PRINT_SIZE_TEXT_VP);
          if(language_change == 0x01)
            rtscheck.RTS_SndData(CORP_WEBSITE_C, WEBSITE_TEXT_VP);
          else
            rtscheck.RTS_SndData(CORP_WEBSITE_E, WEBSITE_TEXT_VP);
          delay(10);
          RTS_SndData(ExchangePageBase + 30, ExchangepageAddr);
        }
      }
      break;

      //case Bedlevel:
      case LevelMode: //0x2012
      {
        if(recdat.data[0] == 1)
        {
          Update_Time_Value = RTS_UPDATE_VALUE;
          InforShowStatus = false;
          //RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
        }
        else if(recdat.data[0] == 2) //调平页面内，点击Z轴补偿“+”功能按钮
        {
          last_zoffset = zprobe_zoffset;
          if (WITHIN((zprobe_zoffset + 0.1), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) 
          {
            zprobe_zoffset = (zprobe_zoffset + 0.1);
            #if HAS_LEVELING
              probe_offset.z += zprobe_zoffset - last_zoffset;
            #endif
            babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);  
            settings.save();
          }
        }
        else if(recdat.data[0] == 3)  //调平页面内，点击Z轴“HOME”按钮
        {
          // Z-axis to home
          planner.synchronize();
          // only for prohibiting to receive massage
          waitway = 6;
          InforShowStatus = false;
          AutohomeKey = true;
          if (!TEST(axis_known_position, X_AXIS) || !TEST(axis_known_position, Y_AXIS))
          {
            queue.enqueue_now_P(PSTR("G28"));
          }
          else
          {
            queue.enqueue_now_P(PSTR("G28 Z0"));
          }
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        }
        else if(recdat.data[0] == 4) //调平页面内，点击Z轴补偿“-”功能按钮
        {
          last_zoffset = zprobe_zoffset;
          if (WITHIN((zprobe_zoffset - 0.1), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) 
          {
            zprobe_zoffset = (zprobe_zoffset - 0.1);
            #if HAS_LEVELING
              probe_offset.z += zprobe_zoffset - last_zoffset;
            #endif
            babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          }
        }
        else if(recdat.data[0] == 5) //在带BL调平页面内点击平台测量按钮
        {
          #ifdef BLTOUCH
            // AutoLevel
            waitway = 3;
            RTS_SndData(1, AutolevelIcon); 
            RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
            queue.enqueue_now_P(PSTR("G29")); 
            // stepper.synchronize();
          #endif
        }
        else if(recdat.data[0] == 6) //在带BL调平页面内点击“辅助调平”按钮
        {
          // Assitant Level
          RTS_SndData(ExchangePageBase + 20, ExchangepageAddr); 
        }
        else if(recdat.data[0] == 7)
        {
          // Autolevel switch
          if(AutoLevelStatus)
          {
            // turn on the Autolevel
            RTS_SndData(3, AUTOLEVEL_SWITCH_ICON_VP);
            AutoLevelStatus = false;
            queue.enqueue_now_P(PSTR("M420 S1"));
            // settings.load();
          }
          else
          {
            // turn off the Autolevel
            RTS_SndData(2, AUTOLEVEL_SWITCH_ICON_VP);
            AutoLevelStatus = true;
            queue.enqueue_now_P(PSTR("M420 S0"));
            // settings.reset();
          }
          BL24CXX_Write(FONT_EEPROM+2, (uint8_t*)&AutoLevelStatus, sizeof(AutoLevelStatus));
        }
        else if(recdat.data[0] == 8) //点击按钮“1”
        {
          // Assitant Level
          if (!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
            queue.enqueue_now_P(PSTR("G1 X135 Y135 F8000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else if(recdat.data[0] == 9) //点击按钮“2”
        {
          // Assitant Level , Front Left 2
          if (!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
            queue.enqueue_now_P(PSTR("G1 X30 Y30 F8000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x0A) //点击按钮“3”
        {
          // Assitant Level , Front Right 3
          if (!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
            queue.enqueue_now_P(PSTR("G1 X255 Y30 F8000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x0B) //点击按钮“4”
        {
          // Assitant Level , Back Right 4
          if (!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
            queue.enqueue_now_P(PSTR("G1 X255 Y255 F8000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x0C) //点击按钮“5”
        {
          // Assitant Level , Back Left 5
          if (!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
            queue.enqueue_now_P(PSTR("G1 X30 Y255 F8000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x0D)
        {
          RTS_SndData(ExchangePageBase + 17, ExchangepageAddr); 
        }

        RTS_SndData(1, MOTOR_STATUS_ICON_VP); 
        Motor_Switch = 1;

        //RTS_SndData(10, FilenameIcon); 
      }
      break;
      
      //移动轴
      //case XYZEaxis:
      case X_MOVE_DATA_KEY: //0x2112
      case Y_MOVE_DATA_KEY: //0x2114
      case Z_MOVE_DATA_KEY: //0x2116
      case AutoZero:        //0x1046
      {
        waitway = 4;
        if(recdat.addr == X_MOVE_DATA_KEY)
        {
          axis = X_AXIS;
          min = X_MIN_POS;
          max = (X_MAX_POS-10);
        }
        else if(recdat.addr == Y_MOVE_DATA_KEY)
        {
          axis = Y_AXIS;
          min = Y_MIN_POS;
          max = (Y_MAX_POS-10);
        }
        else if(recdat.addr == Z_MOVE_DATA_KEY)
        {
          axis = Z_AXIS;
          min  = Z_MIN_POS;
          max  = (Z_MAX_POS-10);
        }
        else if(recdat.addr == AutoZero)
        {
          // autohome
          if(recdat.data[0] == 0x0D)
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G28"));
            InforShowStatus = AutohomeKey = true;
            AutoHomeIconNum = 0;
            Update_Time_Value = 0;
            RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);delay(2);
            RTS_SndData(1, HOME_AUTORUN_VP);delay(2);
            AutoHomeIconNum = 1;
            //RTS_SndData(10,FilenameIcon);
          }
          break;
        }

        current_position[axis] = ((float)recdat.data[0])/10;
        if (current_position[axis] < min) current_position[axis] = min;
        else if (current_position[axis] > max) current_position[axis] = max;

        RTS_SndData(1, MOTOR_STATUS_ICON_VP);
        Motor_Switch = 1;

        RTS_line_to_current(axis);
        RTS_SndData(10*current_position[X_AXIS], X_MOVE_DATA_KEY);
        RTS_SndData(10*current_position[Y_AXIS], Y_MOVE_DATA_KEY);
        RTS_SndData(10*current_position[Z_AXIS], Z_MOVE_DATA_KEY );
        delay(1);
        //RTS_SndData(10, FilenameIcon);
        waitway = 0; 
      }
      break;

      case MovePage: //切换移动轴量程
      {
        if(recdat.data[0]==1)
        {
          RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
        }
        else
        {
          AxisPagenum = recdat.data[0];
          waitway = 0;
        }
      }
      break;

      //case Filement:
      case ChangeFilament: //换料 0x2015
      {
        RTS_SndData((int)PLA_ABSModeTemp, TEMP_WARNING_DATA_VP);

        // unsigned int IconTemp;

        if(recdat.data[0] == 0x01)
        {
          RTS_SndData(ExchangePageBase + 17, ExchangepageAddr); //退出,返回设置界面
          break;
        }
        else
        {
          #if CHECKFILEMENT
            if(RTS_CheckFilement(3)) break;   //进退料进行断料检测
          #endif

          if(recdat.data[0] == 3) //点击“退料”按钮
          {
            current_position[E_AXIS] -= ChangeMaterialbuf[0];
            //RTS_SndData(10, FilenameIcon);

            if(NozzleTempStatus[1] == 0 && thermalManager.temp_hotend[0].celsius < (PLA_ABSModeTemp-5))
            {
              NozzleTempStatus[1] = 1; 
              //RTS_SndData((int)PLA_ABSModeTemp, 0x1020);
              delay(5);
              RTS_SndData(ExchangePageBase + 22, ExchangepageAddr); //调整至，温度不足，请先加热，再换料
              break;
            }
            RTS_SndData(1, MOTOR_STATUS_ICON_VP);
            Motor_Switch = 1;
          }
          else if(recdat.data[0] == 4) //点击“进料”按钮
          {
            current_position[E_AXIS] += ChangeMaterialbuf[0];
            //RTS_SndData(10, FilenameIcon);

            if(NozzleTempStatus[1] == 0 && thermalManager.temp_hotend[0].celsius < (PLA_ABSModeTemp-5))
            {
              NozzleTempStatus[1] = 1; 
              //RTS_SndData((int)PLA_ABSModeTemp, 0x1020);
              delay(5);
              RTS_SndData(ExchangePageBase + 22, ExchangepageAddr); //调整至，温度不足，请先加热，再换料
              break;
            }
            RTS_SndData(1, MOTOR_STATUS_ICON_VP);
            Motor_Switch = 1;
          }
          else if(recdat.data[0] == 5) //温度不足，点击“是”按钮
          {
            NozzleTempStatus[0] = 1;
            // InforShowoStatus = true;
            
            thermalManager.temp_hotend[0].target = (thermalManager.temp_hotend[0].target >= PLA_ABSModeTemp? thermalManager.temp_hotend[0].target:  PLA_ABSModeTemp);
            
            // IconTemp = thermalManager.temp_hotend[0].celsius * 100/thermalManager.temp_hotend[0].target;
            // if(IconTemp >= 100)
            // {
            //   IconTemp = 100;
            // }
            // RTS_SndData(IconTemp, HeatPercentIcon);

            RTS_SndData(thermalManager.temp_hotend[0].celsius,  NOZZLE_TEMP_DATA_VP);
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, NOZZLE_PREHEAT_DATA_VP); 
            delay(5);
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
            break;
          }
          else if(recdat.data[0] == 6)
          {
            NozzleTempStatus[1] = 0;
            RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
            break;
          }
          else if(recdat.data[0] == 0x07) // 进退料加热过程中，退出时，取消
          {
            // InforShowoStatus = true;
            NozzleTempStatus[0] = 0;
            NozzleTempStatus[1] = 0;
            delay(1);

            thermalManager.disable_all_heaters();
            RTS_SndData(0,NOZZLE_PREHEAT_DATA_VP);
            delay(1);
            RTS_SndData(language_change +108 -1,STATUS_DP_CHAR_VP);
            msgStatus = MSG_COOLING;
            RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
            break;
          }
          else if(recdat.data[0] == 0x08)   // not to cancel heating
          {
            break;
          }
          else if(recdat.data[0] == 0x09)
          {
            RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
            break;
          }
        }

        RTS_line_to_current(E_AXIS);
        RTS_SndData(10*ChangeMaterialbuf[0], CHANGE_FILAMENT_UNIT_KEY);
      }
      break;

      case CHANGE_FILAMENT_UNIT_KEY:
      {
        #if CHECKFILEMENT
        /**************checking filement status during changing filement************/
            if(RTS_CheckFilement(3)) break;
        #endif
          
        ChangeMaterialbuf[0] = ((float)recdat.data[0])/10;
      }
      break;

      //case LanguageChoice:
      case LanguageOptions:
      {
        if(recdat.data[0] == 0x0A)              //return
        {
            BL24CXX_Write(FONT_EEPROM, (uint8_t*)&language_change, sizeof(language_change));
            Set_Language(language_change);
            RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
            break;
        }
        else if(recdat.data[0] == 0x0B)         //First boot language confirmation
        {
            BL24CXX_Write(FONT_EEPROM, (uint8_t*)&language_change, sizeof(language_change));
            
            if(language_change < 0x01 || language_change > 0x09)
                break;

            rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
            for(startprogress = 0; startprogress <= 100; startprogress++)
            {
                rtscheck.RTS_SndData(startprogress,START_PROCESS_ICON_VP);
                delay(10);
            }
            Set_Language(language_change);
            rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            rtscheck.RTS_SndData(0,START_PROCESS_ICON_VP);

            break;
        }
        
        language_change = recdat.data[0];
        
        Lcd_Select_Var(language_change,LANGUAGE_1_ICON_VP,9);
        
        delay(10);
        if(IS_SD_INSERTED())
        {
            // for Ready
            RTS_SndData(language_change +63 -1,STATUS_DP_CHAR_VP);
            msgStatus = MSG_READY;
        }
        else
        {
            //  Card Removed
            RTS_SndData(language_change +54 -1,STATUS_DP_CHAR_VP);
            msgStatus = MSG_CARD_PULLOUT;
        }
      }
      break;

      //case No_Filement:
      case NoFilamentContinue: //断料处理
      {
        char comdbuf[30];
        if(recdat.data[0] == 1) //断料提示界面，点击“是”，提示继续打印
        {
          /**************checking filement status during changing filement************/
          if(RTS_CheckFilement(0)) break;

          // check filement before starting to print
          if(FilementStatus[0] == 1)
          {
            queue.enqueue_one_now(cmdbuf);
            queue.enqueue_now_P(PSTR("M24"));

            // clean screen.
            for(int j = 0;j < 10;j++)
            {
              RTS_SndData(0,FILE_SELECT_1_ICON_VP+j);
            }

            int filelen = strlen(CardRecbuf.Cardshowfilename[FilenamesCount]);
            filelen = (TEXTBYTELEN - filelen)/2;
            if(filelen > 0)
            {
              char buf[20];
              memset(buf,0,sizeof(buf));
              strncpy(buf,"         ",filelen);
              strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[FilenamesCount]);
              //RTS_SndData(buf, Printfilename); 
              RTS_SndData(buf, FILE_SELECTED_TEXT_VP);
            }
            else
            {
              //RTS_SndData(CardRecbuf.Cardshowfilename[FilenamesCount], Printfilename);
              RTS_SndData(CardRecbuf.Cardshowfilename[FilenamesCount], FILE_SELECTED_TEXT_VP);
            }
            delay(2);
            RTS_SndData(language_change +72 -1,STATUS_DP_CHAR_VP);// for Heating
            msgStatus = MSG_HEATING;
            //RTS_SndData(1,IconPrintstatus); //加热中，待定
            delay(2);
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
  
            RTS_SndData(feedrate_percentage, PRINT_SPEED_DATA_VP);

            TPShowStatus = InforShowStatus = true;
            PrinterStatusKey[0] = 1;
            PrinterStatusKey[1] = 3;
            // open the key of  checking card in  printing
            CardCheckStatus[0] = 1;
            FilenamesCount = PrintStatue[1] = 0;
            // recover the status waiting to check filements
            FilementStatus[0]  =  0;
          }
          else if(FilementStatus[0] == 2)
          {
            // check filements status during printing
            char pause_str_Z[16];
            char pause_str_E[16];
            memset(pause_str_Z, 0, sizeof(pause_str_Z));
            dtostrf(pause_z, 3, 2, pause_str_Z);
            memset(pause_str_E, 0, sizeof(pause_str_E));
            dtostrf(pause_e, 3, 2, pause_str_E);
            sprintf_P(comdbuf, PSTR("M190 S%i"), tempbed);
            queue.enqueue_one_now(comdbuf);
            memset(comdbuf,0,sizeof(comdbuf));
            sprintf_P(comdbuf, PSTR("M109 S%i"), temphot);
            queue.enqueue_one_now(comdbuf);
            memset(comdbuf,0,sizeof(comdbuf));
            sprintf_P(comdbuf, PSTR("G0 Z%s"), pause_str_Z);
            queue.enqueue_one_now(comdbuf);
            sprintf_P(comdbuf, PSTR("G92 E%s"), pause_str_E);
            queue.enqueue_one_now(comdbuf);
            tempbed = 0;
            temphot = 0;

            card.startFileprint();
            print_job_timer.start();

            //RTS_SndData(1,IconPrintstatus); //加热中，待定
            RTS_SndData(language_change +72 -1,STATUS_DP_CHAR_VP);
            msgStatus = MSG_HEATING;

            PrintStatue[1] = 0;
            PrinterStatusKey[1] = 3;
            // open the key of  checking card in  printing
            CardCheckStatus[0] = 1;

            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);

            // recover the status waiting to check filements
            FilementStatus[0]  =  0;
          }
          else if(FilementStatus[0] == 3)
          {
            RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          }
        }
        else if(recdat.data[0] == 2)  //断料提示界面，点击否
        {
          if(FilementStatus[0] == 1)
          {
            RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
            PrinterStatusKey[0] = 0;
          }
          else if(FilementStatus[0] == 2)
          {
            RTS_SndData(ExchangePageBase + 9, ExchangepageAddr);
          }
          else if(FilementStatus[0] == 3)
          {
            RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          }

          // recover the status waiting to check filements
          FilementStatus[0]  =  0;
        }
      }
      break;
      
      case PwrOffContinue:   //断电续打页面处理
      {
        if(recdat.data[0] == 1)  //点击“是”进行断电续打
        {
          PrinterStatusKey[1] = 3;  // 3 stands for printing

          //RTS_SndData(1, IconPrintstatus); //待定
          RTS_SndData(language_change + 72 - 1, STATUS_DP_CHAR_VP);
          msgStatus = MSG_HEATING;
          RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);

          heat_flag = 1;

          RTS_SndData(zprobe_zoffset*100, Z_OFFSET_DATA_VP);
          RTS_SndData(feedrate_percentage, PRINT_SPEED_DATA_VP);

          recovery.resume(); //直接使用Marlin固件中的恢复打印函数
        }
        else if(recdat.data[0] == 2)  //点击“否”取消断电续打
        {
          InforShowStatus = true;
          TPShowStatus = false;
          Update_Time_Value = RTS_UPDATE_VALUE;
          
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);

          card.stopSDPrint();
          queue.clear();
          quickstop_stepper();
          print_job_timer.stop();
          thermalManager.disable_all_heaters();
          print_job_timer.reset();

          #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
            card.removeJobRecoveryFile();
            recovery.info.valid_head = 0;
            recovery.info.valid_foot = 0;
            recovery.close();
          #endif

          wait_for_heatup = false;
          PrinterStatusKey[0] = 0;
          
          delay(500); // for system
        }
      }
      break;

      //case Filename :
      case SelectFile: //0x2002
      {
        if(recdat.data[0] >= 0x21 && recdat.data[0] <= 0x34)  //在文件选择页面，点击文件图标
        {
          if(IS_SD_INSERTED())  
          {
            if((recdat.data[0] - 0x20) > CardRecbuf.Filesum) break;

            CardRecbuf.recordcount = (recdat.data[0] - 0x21);

            int filelen = strlen(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
            filelen = (TEXTBYTELEN - filelen)/2;
            if(filelen > 0)
            {
              char buf[20];
              memset(buf,0,sizeof(buf));
              strncpy(buf,"         ",filelen);
              strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
            }

            Lcd_Select_Var(recdat.data[0] - 0x20,FILE_SELECT_1_ICON_VP,CardRecbuf.Filesum);

          }
        }
        else if(recdat.data[0] == 0x02) //在文件选中页面中，选中文件后，点击“打印”按钮
        {
          if(IS_SD_INSERTED())
          {
            if(CardRecbuf.recordcount < 0) break;

            char cmd[30];
            char* c;
            sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
            for (c = &cmd[4]; *c; c++) *c = tolower(*c);

            FilenamesCount = CardRecbuf.recordcount;
            memset(cmdbuf,0,sizeof(cmdbuf));
            strcpy(cmdbuf,cmd);

            #if CHECKFILEMENT
              /**************checking filement status during printing beginning ************/
              if(RTS_CheckFilement(1)) break;
            #endif

            queue.enqueue_one_now(cmd);
            queue.enqueue_now_P(PSTR("M24"));

            heat_flag = 1;

            //清除正在打印界面上的文件名图标上的文字
            for(int j = 0;j < 10;j++)
            {
              RTS_SndData(0,FILE_SELECTED_TEXT_VP+j);
            }

            int filelen = strlen(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
            filelen = (TEXTBYTELEN - filelen)/2;
            if(filelen > 0)
            {
              char buf[20];
              memset(buf,0,sizeof(buf));
              strncpy(buf,"         ",filelen);
              strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
              //RTS_SndData(buf, Printfilename);
              RTS_SndData(buf, FILE_SELECTED_TEXT_VP);
            }
            else
            {
              //RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], Printfilename);
              RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], FILE_SELECTED_TEXT_VP);
            }
            delay(2);

            thermalManager.set_fan_speed(0, FanOn);

            // 1 for Heating 
            //RTS_SndData(1,IconPrintstatus); //待定
            RTS_SndData(language_change + 72 - 1,STATUS_DP_CHAR_VP);
            msgStatus = MSG_HEATING;
            delay(2);
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            
            #if ENABLED(BABYSTEPPING)
              RTS_SndData(0, 0x1026); 
            #endif

            feedrate_percentage = 100;
            //RTS_SndData(feedrate_percentage, FeedrateDisplay);
            RTS_SndData(feedrate_percentage, PRINT_SPEED_DATA_VP);
            zprobe_zoffset = last_zoffset = 0;
            //RTS_SndData(zprobe_zoffset*100, 0x1026);
            RTS_SndData(zprobe_zoffset*100, Z_OFFSET_DATA_VP);

            TPShowStatus = InforShowStatus = true;
            PrintModeChange = true;
            PrintStatue[1] = 0;

            // close the checking filements status in printing
            FilementStatus[1]   = 0;
            PrinterStatusKey[0] = 1;
            PrinterStatusKey[1] = 3;

            // open the key of  checking card in  printing
            CardCheckStatus[0] = 1;
            PrintModeTime      = 1;
            Update_Time_Value  = 0;
          }
        }
        else if(recdat.data[0] == 1)   // return to main page
        {
          InforShowStatus = true;
          TPShowStatus = false;
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }   
      }
      break;

      default:
      break;

      memset(&recdat,0 , sizeof(recdat));
      recdat.head[0] = FHONE;
      recdat.head[1] = FHTWO;
    }
  }
  
  void EachMomentUpdate()
  {
    millis_t ms = millis();
    if(ms > next_rts_update_ms && InforShowStatus)
    {
      static unsigned int last_cardpercentValue = 101;

      //显示打印时间
      duration_t elapsed = print_job_timer.duration();   // print timer 
      rtscheck.RTS_SndData(elapsed.value/3600,PRINT_TIMEHOUR_DATA_VP);
      rtscheck.RTS_SndData((elapsed.value%3600)/60,PRINT_TIMEMIN_DATA_VP);

      //显示打印百分比
      if(card.isPrinting() && last_cardpercentValue != card.percentDone()) // pint percent
      {
        if((unsigned int) card.percentDone() > 0)
        {
          Percentrecord = card.percentDone()+1;
          rtscheck.RTS_SndData((unsigned int)Percentrecord ,START_PROCESS_ICON_VP);
        }
        else
        {
          rtscheck.RTS_SndData(0,START_PROCESS_ICON_VP);
        }
        rtscheck.RTS_SndData((unsigned int) card.percentDone(),PRINT_PERCENT_DATA_VP);
        last_cardpercentValue = card.percentDone();
      }

      //对目前的热床温度和喷头温度进行刷新显示
      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius,NOZZLE_TEMP_DATA_VP);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius,BED_TEMP_DATA_VP);

      //热床、喷头的目标温度与当前温度不一致的情况下进行处理
      if( (last_target_temperature_bed != thermalManager.temp_bed.target) || (last_target_temperature[0] != thermalManager.temp_hotend[0].target))
      {
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target,NOZZLE_PREHEAT_DATA_VP);
        rtscheck.RTS_SndData(thermalManager.temp_bed.target,BED_PREHEAT_DATA_VP);

        if(card.isPrinting())
        {
          // keep the icon
        }
        else if(((last_target_temperature_bed < thermalManager.temp_bed.target) && (thermalManager.temp_bed.celsius < thermalManager.temp_bed.target)) || 
            ((last_target_temperature[0] < thermalManager.temp_hotend[0].target) && (thermalManager.temp_hotend[0].celsius < thermalManager.temp_hotend[0].target)))
        {
          //rtscheck.RTS_SndData(1,IconPrintstatus); //正在加热
          rtscheck.RTS_SndData(language_change +72 -1,STATUS_DP_CHAR_VP);
          msgStatus = MSG_HEATING;

          Update_Time_Value = 0;
          PrinterStatusKey[1] =(PrinterStatusKey[1] == 0? 1 : PrinterStatusKey[1]);
        }
        else if(((last_target_temperature_bed > thermalManager.temp_bed.target) && (thermalManager.temp_bed.celsius > thermalManager.temp_bed.target)) || 
            ((last_target_temperature[0] > thermalManager.temp_hotend[0].target) && (thermalManager.temp_hotend[0].celsius > thermalManager.temp_hotend[0].target)))
        {
          if(!state_stop_flag)
          {
            //rtscheck.RTS_SndData(8,IconPrintstatus); //正在冷却 待定
            rtscheck.RTS_SndData(language_change +108 -1,STATUS_DP_CHAR_VP);
            msgStatus = MSG_COOLING;
          }
          state_stop_flag = 0;
          Update_Time_Value = 0;
          PrinterStatusKey[1] =(PrinterStatusKey[1] == 0? 2 : PrinterStatusKey[1]);
        }
        last_target_temperature_bed = thermalManager.temp_bed.target;
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
      }
      // 当显示"正在降温。。。"时，当温度到达，取消显示
      if (msgStatus == MSG_COOLING)
      {
        if (thermalManager.temp_bed.celsius < 35 && thermalManager.temp_hotend[0].celsius < 35)
        {
          rtscheck.RTS_SndData(language_change +63 -1,STATUS_DP_CHAR_VP);
          msgStatus = MSG_READY;
        }
      }

      // statuse of loadfilement and unloadfinement when temperature is less than
      if(NozzleTempStatus[0] || NozzleTempStatus[2])
      {
        // unsigned int IconTemp;

        // // heat percent
        // if(thermalManager.temp_hotend[0].target)
        //   IconTemp = thermalManager.temp_hotend[0].celsius * 100/thermalManager.temp_hotend[0].target;
        // else 
        //   IconTemp = 100;

        // if(IconTemp >= 100)
        // {
        //   IconTemp = 100;
        // }
        // rtscheck.RTS_SndData(IconTemp, HeatPercentIcon);  //待定

        //喷头换料处理段
        if(thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target && NozzleTempStatus[0])
        {
          NozzleTempStatus[1] = 0;
          NozzleTempStatus[0] = 0;
          // change the material value
          rtscheck.RTS_SndData(10*ChangeMaterialbuf[0], FilementUnit1);   //待定
          // rtscheck.RTS_SndData(10*ChangeMaterialbuf[1], FilementUnit2);
         
          rtscheck.RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);  //切换页面，调转至有触控功能的换料页面

          RTS_line_to_current(E_AXIS);
          // delay(current_position[E_AXIS] * 1000);
        }
        else if(thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target && NozzleTempStatus[2])
        {
          // SERIAL_ECHOPAIR("\n ***NozzleTempStatus[2] =",(int)NozzleTempStatus[2]);
          startprogress = NozzleTempStatus[2] = 0;
          TPShowStatus = true;
          rtscheck.RTS_SndData(4, ExchFlmntIcon);
        }
        else if(NozzleTempStatus[2])
        {
          rtscheck.RTS_SndData((startprogress++)%5, ExchFlmntIcon);
        }
      }

      if(AutohomeKey)
      {
        // AutoZeroIcan GIF
        //rtscheck.RTS_SndData(AutoHomeIconNum++,AutoZeroIcon);
        rtscheck.RTS_SndData(AutoHomeIconNum++,HOME_AUTORUN_VP);
        if(AutoHomeIconNum >= 9)
        {
          AutoHomeIconNum = 1;
        }
      }

      //暂停状态下回X轴Y轴回原点
      if(pause_action_flag && (PrintStatue[1] == 1) && printingIsPaused() && !planner.has_blocks_queued()) 
      {
        pause_action_flag = false;
        queue.enqueue_now_P(PSTR("G28XY"));
      }
        
      // }

      next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
    }
  }

  //在主循环中进行串口屏数据处理
  void RTSUpdate()
  {
    rtscheck.RTS_SDCardUpate(); //检测SD卡状态

    #if CHECKFILEMENT
      // checking filement status during printing
      if(FilementStatus[1] == 2 && true == card.isPrinting()) // 3DPrint is printing
      {
        if(1 == READ(CHECKFILEMENT_PIN))
        {
          Checkfilenum++;
          delay(5);
          if(Checkfilenum>50)
          {
            rtscheck.RTS_SndData(ExchangePageBase + 43, ExchangepageAddr);
            
            waitway = 5;
            pause_z = current_position[Z_AXIS];
            pause_e = current_position[E_AXIS] - 2;

            #if ENABLED(POWER_LOSS_RECOVERY)
              if (recovery.enabled) recovery.save(true, false);
            #endif

            card.pauseSDPrint();
            print_job_timer.pause();

            if(!temphot)
              temphot = thermalManager.degTargetHotend(0);
            if(!tempbed)
              tempbed = thermalManager.degTargetBed();
            // thermalManager.setTargetHotend(0, 0);
            //thermalManager.setTargetBed(0);

            // for returning the corresponding page

            PrintStatue[1] = 1; //注释

            pause_action_flag = true;
            // no filements during printing
            FilementStatus[0] = 2;
            Checkfilenum = 0;
            FilementStatus[1] = 0;
            PrinterStatusKey[1] = 4;

            TPShowStatus = false;
            Update_Time_Value = 0;
          }
        }
      }
    #endif

    EachMomentUpdate();
    
    //断电续打处理函数，只进入一次
    if(!power_off_type_yes)
    {
      power_off_type_yes = 1;

      if(recovery.valid() && RTS_lcd_sd_status)
      {
        for (uint16_t i = 0; i < CardRecbuf.Filesum ; i++)
        {
          if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1])) // Resume print before power failure while have the same file
          {
            InforShowStatus = true;
            int filelen = strlen(CardRecbuf.Cardshowfilename[i]);
            filelen = (TEXTBYTELEN - filelen)/2;
            if(filelen > 0)
            {
              char buf[20];
              memset(buf,0,sizeof(buf));
              strncpy(buf,"         ",filelen);
              strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[i]);
              rtscheck.RTS_SndData(buf, FILE_SELECTED_TEXT_VP);
            }
            else
            {
              // filenames
              rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i],FILE_SELECTED_TEXT_VP);
            }
          }
          else
          {
            power_off_continue = 0;
          }

          power_off_continue = 1;
        }
        
        if(power_off_continue)
        {
          rtscheck.RTS_SndData(ExchangePageBase + 37, ExchangepageAddr); 
        }
      }
    }

    // wait to receive massage and response
    if(rtscheck.RTS_RecData() > 0)
    {
      rtscheck.RTS_HandleData();
    }
  }

  void Lcd_Select_Var(unsigned char Num,int BeginAddr,unsigned char TotalNum)
  {
    for(unsigned char i = 1;i <= TotalNum;i++)
    {
        if(i == Num)
          rtscheck.RTS_SndData(1, BeginAddr+i-1);
        else
          rtscheck.RTS_SndData(0, BeginAddr+i-1);
    }
  }

#endif


