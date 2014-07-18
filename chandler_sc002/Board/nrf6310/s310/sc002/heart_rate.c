
#include "heart_rate.h"
#include "stdlib.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "app_timer.h"
#include "hrm_tx.h"

// Some values used to simulate measurements
#define BPM_DEFAULT                           0

// HR  unit: ms
#define HR80				750
#define HR100				600
#define HR120				500
#define HR140				428
#define HR150				400
#define HR160				375
#define HR180				330
#define HR40				1505
#define HR60				1000
#define HR220				270
#define HR240				250
#define SLOTGAP			7


static uint8_t heartRateBpm = BPM_DEFAULT;


static uint32_t HRTimePre=0,RgHRTimePre=0;
static uint16_t HRInterval,RgHRInterval,RgHRIntervalPre;
static uint8_t count=0;
static uint16_t delta=0;
static uint8_t SynCount=0;
static uint16_t HRSum=0,HRGap=0,HRInt=0;
static uint16_t HR[8],RrInterval[IntervalBufMax];
static uint16_t GapLimit;
static uint16_t SynGap;
static uint16_t HRIntervalMax,HRIntervalMin;
static uint8_t SynOk=0;
static uint32_t HRBuf[64],RgHRBuf[64];
static uint8_t HRBufEndIndx,HRBufProcIndx=0;
static uint8_t rrIntervalCnt=0;
static uint8_t noiseCnt=0,noiseTimer=0,noiseFlg=0;
static uint8_t RgTempCnt=0;


void checkHRRegular(void)
{
  uint8_t i,temp,periodic=0;
  uint32_t RgHRTime;
  
  RgHRTime = RgHRBuf[HRBufProcIndx] ;
  RgHRInterval = (uint16_t)(RgHRTime - RgHRTimePre);
  
  if((RgHRInterval > HR220) && (RgHRInterval < HR40)) {
    if(RgHRInterval > HR80) SynGap = 150;
    else if(RgHRInterval > HR120) SynGap = 100;
    else if(RgHRInterval > HR150) SynGap = 50;
    else SynGap = 30;
    
    if(abs(RgHRInterval-RgHRIntervalPre)<SynGap) {
      SynCount++;  
      if(SynOk){
        if(SynCount >= 15){
           periodic = 2; 
           //noHRTimer = 0;
        }
      }
      else{
        if(SynCount>=5){
          SynOk = 1;
          periodic = 1;
        }
      }
    }  
    else{
      SynCount = 0;
    }
    if(periodic==1){
       SynCount = 0;
       for(i=0;i<8;i++){
         HR[i] = RgHRInterval;
       }
       temp = RgHRInterval/8;
       HRIntervalMax = RgHRInterval + temp;
       HRIntervalMin = RgHRInterval - temp;
       delta = 0;
       //noHRTimer = 0;
       noiseFlg = 0;
    }
    else if(periodic==2){
     
      temp = HRInt/4;
      SynCount = 0;
      RgTempCnt = 4;
      temp = HRInterval/8;
      HRIntervalMax = RgHRInterval + temp;
      HRIntervalMin = RgHRInterval - temp;
      delta = 0;
      //noHRTimer = 0;
      noiseFlg = 0;
      noiseCnt = 0;
    }
  }
  RgHRTimePre = RgHRTime;
  RgHRIntervalPre = RgHRInterval;
}
/**@brief 心跳偵測處理
 *
 * @details 
 */
void  F_HearRateProcess(void)
{
		antHeartRatePage4Data_t antPage4Data;
   uint8_t i;
   uint32_t HRTime,tmp;
   uint16_t IntervalMaxTemp,IntervalMinTemp;
	
   HRBufProcIndx = 0;

   noiseTimer++;
   if(noiseTimer>=10)
     noiseFlg=0;

   while(HRBufProcIndx<HRBufEndIndx){
     
     checkHRRegular();
     HRTime = HRBuf[HRBufProcIndx] ;
     HRBufProcIndx++;
     HRInterval = (uint16_t)(HRTime - HRTimePre);		 
     if(SynOk) {
       IntervalMaxTemp = HRIntervalMax + delta;
       IntervalMinTemp = HRIntervalMin - delta;
       if(IntervalMaxTemp>HR40) IntervalMaxTemp = HR40;
       if(IntervalMinTemp<HR220) IntervalMinTemp = HR220;
       
       if((HRInterval<IntervalMaxTemp) && (HRInterval>IntervalMinTemp)){
         rrIntervalCnt++;
         if(rrIntervalCnt>8)
           rrIntervalCnt = 8;
         
         count++;
         if(count>4){
           delta = 0;
           //noHRTimer = 0;
         }
         for(i=0;i<7;i++){        //shift
           HR[7-i] = HR[6-i]; 
           RrInterval[7-i] = RrInterval[6-i];
         }
         HR[0] = HRInterval;
         tmp = HRInterval;
         tmp <<= 10;                //trans to 1/1024
         tmp /= 1000;
         RrInterval[0] = (uint16_t)tmp;     
         
         HRSum = 0;
         for(i=0;i<8;i++){        //summing
           HRSum += HR[i];
         }
         HRInt = HRSum/8;  
         
         HRGap = HRInt/8;
      
         if(RgTempCnt==0){
            HRIntervalMax = HRInt + HRGap;
            HRIntervalMin = HRInt - HRGap;
            if(RgTempCnt!=0)
              RgTempCnt--;
            
            if(HRInt > HR100){       //broaden range
              HRIntervalMax += HRGap;
              HRIntervalMin -= HRGap;
            }
         }
                  
         heartRateBpm = (uint8_t) (60000 / HRInt);
			 //=============================
			 // ANT 資料
				if(HRTime > HRTimePre) {
					antPage4Data.HeartBeatEventTime = HRTime;
					antPage4Data.PreviousHeartBeatEventTime = HRTimePre;
				} else {
					antPage4Data.HeartBeatEventTime = HRTimePre;
					antPage4Data.PreviousHeartBeatEventTime = HRTime;
				}
				antPage4Data.ComputedHeartRate = heartRateBpm;
				F_UpAntHeartRatePage4Data(antPage4Data);
			 //============================= 
         if(HRIntervalMax>HR40) HRIntervalMax = HR40;
         if(HRIntervalMin<HR220) HRIntervalMin = HR220;
         HRTimePre = HRTime; 
      
       }
       else if(HRInterval>HRIntervalMax){
         count=0;
         if(noiseFlg!=1)
            delta += SLOTGAP;
         else if(HRInt >= HR60)
            delta += SLOTGAP;
         else 
            delta += (SLOTGAP/2);
        
         if(HRInt > HR60)  GapLimit = HRInt/4;
         else if (HRInt >= HR160) GapLimit = HRInt/8;
         else if (HRInt >= HR180) GapLimit = HRInt/16;
         else GapLimit = 10;
         
         if(delta > GapLimit) delta = GapLimit;
         
         HRTimePre = HRTime; 
       }
       else if(HRInterval<HRIntervalMin) {
         noiseCnt++;
         noiseTimer=0;
         if(noiseCnt>=5)
            noiseFlg = 1;
       }
     }
     else{
       HRTimePre = HRTime;
     }
   }
   HRBufProcIndx = 0;
   HRBufEndIndx = 0;
   for(i=0;i<64;i++){
     HRBuf[i] = 0;
     RgHRBuf[i] = 0;
   }	 
	
}

/**@brief 心跳訊號時間儲存
 *
	PRESCALER = 0
	F = 32.768KHz/ PRESCALER + 1 
	T = 30.517uS
 * @details 
 */
void  F_HearRateSignalTimeSave(void)
{
		static uint32_t Rtc1TimeCntNow,Rtc1TimeCntBefore;
		uint32_t heartTime,Temp;
		app_timer_cnt_get(&Rtc1TimeCntNow);
		if(Rtc1TimeCntNow>Rtc1TimeCntBefore) {
			heartTime = Rtc1TimeCntNow - Rtc1TimeCntBefore;
		} else {
			heartTime = Rtc1TimeCntBefore - Rtc1TimeCntNow;
		}
		if(heartTime >= 8936) { //  
			Temp = (Rtc1TimeCntNow / (1000/30.517)); // 儲存時間轉換為 mS
			HRBuf[HRBufEndIndx] = Temp;
			RgHRBuf[HRBufEndIndx] = Temp;
			HRBufEndIndx++;
			if(HRBufEndIndx>=64)
				HRBufEndIndx = 0;
			Rtc1TimeCntBefore = Rtc1TimeCntNow;
		}
}
/**@brief  讀取HeartRateData
 *
 * @details 
 */
void F_ReadHeartRateData(HeartRateData_t *heartRateData)
{
	uint8_t i;
	for(i=0 ; i<IntervalBufMax ; i++) {
		heartRateData ->RrInterval[i] = RrInterval[i];
	}
	heartRateData ->rrIntervalCnt = rrIntervalCnt;
	heartRateData ->heartRate = heartRateBpm;
}
/**@brief 清除IntervalCnt
 *
 * @details 
 */
void F_ClearIntervalCnt(void)
{
	rrIntervalCnt = 0;
}
			/*
		static uint32_t Rtc1TimeCnt1,Rtc1TimeCnt2;
		uint32_t heartTime,heartRateTemp;
		antHeartRatePage4Data_t antPage4Data;
		uint8_t	heartRate;

						app_timer_cnt_get(&Rtc1TimeCnt1);
						if(Rtc1TimeCnt1>Rtc1TimeCnt2) {
							heartTime = Rtc1TimeCnt1 - Rtc1TimeCnt2;
							antPage4Data.HeartBeatEventTime = Rtc1TimeCnt1;
							antPage4Data.PreviousHeartBeatEventTime = Rtc1TimeCnt2;
						} else {
							heartTime = Rtc1TimeCnt2 -Rtc1TimeCnt1;
							antPage4Data.HeartBeatEventTime = Rtc1TimeCnt2;
							antPage4Data.PreviousHeartBeatEventTime = Rtc1TimeCnt1;
						}
						heartRateTemp = 	60000000000 / (heartTime * 30517);
						if((heartRateTemp >= 40) && (heartRateTemp <= 220)) {
						//if((heartTime <= 49152) && (heartTime >= 8936)) {
							heartRate = (uint8_t)heartRateTemp;
							F_HearRateProcess(&heartRate);
							R_heartRate = heartRate;
							//==========
							antPage4Data.ComputedHeartRate = R_heartRate;
							F_UpAntHeartRatePage4Data(antPage4Data);
							//==========
							Rtc1TimeCnt2 = Rtc1TimeCnt1;
						}
		//===========================================================
void  F_HearRateProcess(uint8_t *RealHeartRate)
{
    static uint8_t	b_W_Pu1int;
    static uint8_t	R_DisplayHeartRate;
    static uint8_t	R_W_pluseIndex,R_W_PluOld,R_W_pubf[8];
    uint16_t    i;
    uint8_t     k,Temp,m;

		if(R_DisplayHeartRate == 0) {
			b_W_Pu1int = 0;
			if(R_W_PluOld == 0){
				R_W_PluOld = (*RealHeartRate);
			} else {
				if((*RealHeartRate) > R_W_PluOld)
					i = (*RealHeartRate) - R_W_PluOld;
					else
						i = R_W_PluOld - (*RealHeartRate);
				//===================
				if(i<10) {
						b_W_Pu1int=1;
				}
				R_W_PluOld = (*RealHeartRate);
			}
		} else {
				b_W_Pu1int = 1;
		}
    //=========================================
    //
    if(b_W_Pu1int == 1) {
      b_W_Pu1int = 0;
      if(R_DisplayHeartRate == 0) {
        R_W_pluseIndex=0;
        if((*RealHeartRate) > 99) {
          R_DisplayHeartRate = 99;
          R_W_pubf[0] = R_DisplayHeartRate;
          R_W_pubf[1] = R_DisplayHeartRate;
          R_W_pubf[2] = R_DisplayHeartRate;
          R_W_pubf[3] = R_DisplayHeartRate;	
        } else if((*RealHeartRate) <70) {
          R_DisplayHeartRate = 70;			
          R_W_pubf[0] = R_DisplayHeartRate;
          R_W_pubf[1] = R_DisplayHeartRate;
          R_W_pubf[2] = R_DisplayHeartRate;
          R_W_pubf[3] = R_DisplayHeartRate;
        } else {
					R_DisplayHeartRate = (*RealHeartRate);
          R_W_pubf[0] = R_DisplayHeartRate;
          R_W_pubf[1] = R_DisplayHeartRate;
          R_W_pubf[2] = R_DisplayHeartRate;
          R_W_pubf[3] = R_DisplayHeartRate;
        }
      } else {
        k=0;
        m=0;
						//===============
						if(R_DisplayHeartRate > (*RealHeartRate)){
							k = R_DisplayHeartRate - (*RealHeartRate);
						} else {
							m = (*RealHeartRate) - R_DisplayHeartRate;
						}
						//=================
						#define EorrNumVal		10	
						if(k > EorrNumVal) {
							Temp = R_DisplayHeartRate - EorrNumVal;
						} else	if(m > EorrNumVal) {
							Temp = R_DisplayHeartRate + EorrNumVal;
						} else
							 Temp = (*RealHeartRate);
						R_W_pubf[R_W_pluseIndex] = Temp;
						R_W_pluseIndex++;
						if(R_W_pluseIndex >= 8)
							R_W_pluseIndex=0;
						if(R_W_pluseIndex == 0) {
							i = 0;
							for(Temp = 0;Temp<=8; Temp++) {
								i = i + R_W_pubf[Temp];	
							}
							i = i / 8;
							R_DisplayHeartRate = i;	
						}
      }
    }
		*RealHeartRate = R_DisplayHeartRate;
}
						
						
			*/
