/*
 * SCANDINOVA device support
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include <alarm.h>
#include <epicsStdio.h>
#include <devCommonGpib.h>
#include <menuFtype.h>
#include <errlog.h>
#include <link.h>
#include <dbCommon.h>
#include <dbDefs.h>
#include <aiRecord.h>
#include <osiUnistd.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <epicsAssert.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsExport.h>

#include "devSCANDINOVA.h"

SCANDINOVA_INFO SDN[MAX_SCANDINOVA_CNT];

/******************************************************************************
 *
 * The following define statements are used to declare the names to be used
 * for the dset tables.   
 *
 * A DSET_AI entry must be declared here and referenced in an application
 * database description file even if the device provides no AI records.
 *
 ******************************************************************************/
#define DSET_AI     devAiSCANDINOVA
#define DSET_AO     devAoSCANDINOVA
#define DSET_BI     devBiSCANDINOVA
#define DSET_BO     devBoSCANDINOVA
#define DSET_EV     devEvSCANDINOVA
#define DSET_LI     devLiSCANDINOVA
#define DSET_LO     devLoSCANDINOVA
#define DSET_MBBI   devMbbiSCANDINOVA
#define DSET_MBBID  devMbbidSCANDINOVA
#define DSET_MBBO   devMbboSCANDINOVA
#define DSET_MBBOD  devMbbodSCANDINOVA
#define DSET_SI     devSiSCANDINOVA
#define DSET_SO     devSoSCANDINOVA
#define DSET_WF     devWfSCANDINOVA

#include <devGpib.h> /* must be included after DSET defines */

#define TIMEOUT     1.0    /* I/O must complete within this time */
#define TIMEWINDOW  2.0    /* Wait this long after device timeout */

#define MAX_TOKEN 20
#define MAX_LEN 512

/******************************************************************************
 * String arrays for EFAST operations. The last entry must be 0.
 *
 * On input operations,only as many bytes as are found in the string array
 * elements are compared.  Additional bytes are ignored.
 * The first matching string  will be used as a match.
 *
 * For the input operations,the strings are compared literally!  This
 * can cause problems if the instrument is returning things like \r and \n
 * characters.  When defining input strings so you include them as well.
 ******************************************************************************/

static char *userOffOn[] = {"USER OFF;","USER ON;",0};

/******************************************************************************
 * Array of structures that define all GPIB messages
 * supported for this type of instrument.
 ******************************************************************************/
static int procPing0Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3);
static int procPing1Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3);
static int procPing2Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3);
static int procPing3Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3);
static int convertAiData(struct gpibDpvt *pdpvt, int P1, int P2, char **P3);
static int convertAoData(struct gpibDpvt *pdpvt, int P1, int P2, char **P3);
static int convertMbbiData(struct gpibDpvt *pdpvt, int P1, int P2, char **P3);
static void runAutoDriveThreadFunc(void *lParam);
int changeMode(int nDevIdx, int nMode);
int setHv(int nDevIdx, double dbSetpoint);
int increaseHv(SCANDINOVA_AUTO_DRIVE_INFO *p);
int hwControlset(int nDevIdx, int nMode);

static struct gpibCmd gpibCmds[] = {
	// 0: ping 0
	//{&DSET_BI, GPIBCVTIO, IB_Q_HIGH, NULL, NULL, 0, 256, procPing0Msg, 0, 0, NULL, NULL, NULL},
	{&DSET_BI, GPIBREAD, IB_Q_HIGH, "{P|000}", NULL, 0, 256, 
		procPing0Msg, 0, 0, NULL, NULL, "}"},
	
	// 1: ping 1
	{&DSET_BI, GPIBREAD, IB_Q_HIGH, "{P|001}", NULL, 0, 256, 
		procPing1Msg, 0, 0, NULL, NULL, "}"},
	// 2: AI state set
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	// 3: AO state set
	//{&DSET_AO, GPIBCVTIO, IB_Q_HIGH, NULL, NULL, 0, 32, 
	//	sendAoCommand, 'W', 0x001, NULL, NULL, NULL},
	{&DSET_LO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|001|%04X}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 4 ~ 19
	// 4: AI state read 
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	// 4~19 END
	// 20~28
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	// 20~28 END
	
	// 29~30 ping2, ping3
	{&DSET_BI, GPIBREAD, IB_Q_HIGH, "{P|002}", NULL, 0, 256, 
		procPing2Msg, 0, 0, NULL, NULL, "}"},
	{&DSET_BI, GPIBREAD, IB_Q_HIGH, "{P|003}", NULL, 0, 256, 
		procPing3Msg, 0, 0, NULL, NULL, "}"},	// 29~30 END
	
	// 31~35 ping2
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_MBBI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertMbbiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	// 31~35 END
	
	// 36 control word set
	{&DSET_LO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|003|%04X}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 37 PrfSet
	{&DSET_LO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|12C|%ld}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 38 standby curr set
	{&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|2BE|%.4f}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 39 hvps voltage set
	{&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|3EB|%.4f}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 40 Plswth set
	{&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|4B3|%.4f}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 41 magps1 curr set
	{&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|642|%.4f}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 42 magps2 curr set
	{&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|6A6|%.4f}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 43 magps3 curr set
	{&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|70A|%.4f}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},
	// 44 magps4 curr set
	{&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "{W|76E|%.4f}", 0, 32,
		NULL, 0, 0, NULL, NULL, NULL},

	// 45,46 tunnel vacuum1 high/low limit
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	

	// 47 ~ 59 vacuum auto drive get
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	
	{&DSET_AI, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32, 
		convertAiData, 0, 0, NULL, NULL, NULL},	

	// 60 ~ 72 vacuum auto drive set
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	{&DSET_AO, GPIBSOFT, IB_Q_HIGH, NULL, NULL, 0, 32,
		convertAoData, 0, 0, NULL, NULL, NULL},
	
};

/* The following is the number of elements in the command array above.  */
#define NUMPARAMS sizeof(gpibCmds)/sizeof(struct gpibCmd)

/******************************************************************************
 * Initialize device support parameters
 *
 *****************************************************************************/
static long init_ai(int parm)
{
	char thName[256];
	int i,nDevIdx;
    if(parm==0) {
        devSupParms.name = "devSCANDINOVA";
        devSupParms.gpibCmds = gpibCmds;
        devSupParms.numparams = NUMPARAMS;
        devSupParms.timeout = TIMEOUT;
        devSupParms.timeWindow = TIMEWINDOW;
        devSupParms.respond2Writes = -1;
	
		memset(&SDN,0x00,sizeof(SCANDINOVA_INFO)*MAX_SCANDINOVA_CNT);
		
		// auto drive

		for(nDevIdx=0;nDevIdx!=MAX_SCANDINOVA_CNT;++nDevIdx)
		{
			SDN[nDevIdx].nDevIdx = nDevIdx;
			for(i=0;i!=MAX_SCANDINOVA_VACUUM_COUNT;++i)
			{
				if(i==0)
				{
					SDN[nDevIdx].SADI[i].bUse = 1;
					SDN[nDevIdx].SADI[i].dbVacuum = &SDN[i].dbSolonoidPs2CurrRead;
					SDN[nDevIdx].SADI[i].dbTripHighLimit = 5.2;
					SDN[nDevIdx].SADI[i].dbAlarmHighLimit = 4.8;
					SDN[nDevIdx].SADI[i].dbAlarmLowLimit = 3.6;
					SDN[nDevIdx].SADI[i].dbTripLowLimit = 3.5;
					SDN[nDevIdx].SADI[i].dbHVRampSpeed = 1; // voltage
					SDN[nDevIdx].SADI[i].dbHVRampCheckTime = 60; // second 		
					SDN[nDevIdx].SADI[i].dbHVMaxPoint = 1290.0;

					SDN[nDevIdx].SADI[i].dbTripBlockingTime = 300;
					SDN[nDevIdx].SADI[i].dbAlarmBlockingTime = 30;
					SDN[nDevIdx].SADI[i].dbAlarmDecreaseTime = 300;		// (unit: sec)

					SDN[nDevIdx].SADI[i].dbHVTripGain = 90;		// percent
					SDN[nDevIdx].SADI[i].dbHVAlarmGain = 100;	// percent
					
					SDN[nDevIdx].SADI[i].nPriority = MASTER;
				}
				else
				{
					SDN[nDevIdx].SADI[i].dbVacuum = &SDN[i].dbSolonoidPs2CurrRead;
					SDN[nDevIdx].SADI[i].nPriority = SLAVE;
					SDN[nDevIdx].SADI[i].bUse = 0;
				}
    
				SDN[nDevIdx].SADI[i].nIdx = i;
				SDN[nDevIdx].SADI[i].nParentId = nDevIdx;
				epicsTimeGetCurrent(&SDN[nDevIdx].SADI[i].tLastIncrease);

				sprintf(thName,"AUTODRIVE#%d_%d",nDevIdx,i);
				epicsThreadCreate(thName,epicsThreadPriorityHigh,epicsThreadGetStackSize(epicsThreadStackSmall),
						(EPICSTHREADFUNC)runAutoDriveThreadFunc,&SDN[nDevIdx].SADI[i]);
				epicsPrintf("creat the scandinova auto drive func... [%d][%d] - %d,%d\n",nDevIdx,i
						,SDN[nDevIdx].SADI[i].nIdx,SDN[nDevIdx].SADI[i].bUse);
			}
		}
    }
    return(0);
}

static int procPing0Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3)
{

	  asynUser *pasynUser = pdpvt->pasynUser;
	  struct biRecord *pBi= (struct biRecord *)pdpvt->precord;
	  struct link *pLink = (struct link *)&pBi->inp;
	  asynOctet *pasynOctet = pdpvt->pasynOctet;
	  void *asynOctetPvt = pdpvt->asynOctetPvt;

	  int j;
	  size_t nRet;
	  char recvBuf[300];
	  char result[MAX_TOKEN][MAX_LEN];
	  int nCnt;
	  int nIdx = 0;
	  char *beg,*end;
	  int nAddr;
	  int nDevIdx;

	  nDevIdx = pLink->value.gpibio.link;
	  nAddr = pLink->value.gpibio.addr;
	  strncpy(recvBuf,&pdpvt->msg[0],pdpvt->msgInputLen);
	  nRet = pdpvt->msgInputLen;
	  
	  
//	  if ((pasynOctet->write(asynOctetPvt,pasynUser,"{P|000}",sizeof"{P|000}" - 1,&nRet) != asynSuccess)
//		|| (pasynOctet->read(asynOctetPvt,pasynUser,recvBuf,sizeof recvBuf - 1,&nRet,NULL) != asynSuccess)) 
//	  {
//		  epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"PING I/O failed[%d]\n",nRet);
//		  pBi->val = 0;
//		  return -1;
//	  }
	  
	  
	  recvBuf[nRet] = '\0';
	  
	  if(strncmp(recvBuf,"{p|000",6) != 0)
		  return 0;

	  nCnt = 0;
	  beg = &recvBuf[1];
	  end = &recvBuf[nRet];
	  while(beg<end)
	  {
		  if(*beg != '|')
		  {
			  result[nCnt][nIdx++] = *beg;
		  }
		  else
		  {
			  result[nCnt][nIdx] = '\0';
			  nIdx = 0;
			  ++nCnt;
		  }
		  ++beg;
	  }
	  ++nCnt;

//#define DEBUG_PING0
#ifdef DEBUG_PING0
	  epicsPrintf("source: %s\n",recvBuf);
	  epicsPrintf("mySplit count = %d\n",nCnt);
	  for(nIdx=0;nIdx<nCnt;++nIdx)
		  epicsPrintf("[%d]: %s\n",nIdx,result[nIdx]);
#endif

	SDN[nDevIdx].dbStateSet = strtoul(result[3],NULL,16);
	SDN[nDevIdx].dbStateRead = strtoul(result[2],NULL,16);
	SDN[nDevIdx].dbFilamentVoltRead = atof(result[4]);
	SDN[nDevIdx].dbFilamentCurrRead = atof(result[5]);
	SDN[nDevIdx].dbCtRead = atof(result[6]);
	SDN[nDevIdx].dbCvdRead = atof(result[7]);
	SDN[nDevIdx].dbCtArcPerSecondRead = strtoul(result[8],NULL,16);
	SDN[nDevIdx].dbCvdArcPerSecondRead = strtoul(result[9],NULL,16);
	SDN[nDevIdx].dbPrfRead = atof(result[10]);
	SDN[nDevIdx].dbPlswthRead = atof(result[11]);
	SDN[nDevIdx].dbPowRead = atof(result[12]);
	SDN[nDevIdx].dbHVPSVoltRead = atof(result[13]);
	SDN[nDevIdx].dbHVPSVoltSet = atof(result[14]);
	SDN[nDevIdx].dbPlswthSet = atof(result[15]);
	SDN[nDevIdx].dbPrfSet = atof(result[16]);
	SDN[nDevIdx].dbRemainingTime = strtoul(result[17],NULL,16);
	SDN[nDevIdx].dbAccessLevel = strtoul(result[18],NULL,16);
	pBi->val = 1;
	  return 0;
}

static int procPing1Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3)
{
	  asynUser *pasynUser = pdpvt->pasynUser;
	  struct biRecord *pBi= (struct biRecord *)pdpvt->precord;
	  struct link *pLink = (struct link *)&pBi->inp;
	  asynOctet *pasynOctet = pdpvt->pasynOctet;
	  void *asynOctetPvt = pdpvt->asynOctetPvt;

	  int j;
	  size_t nRet;
	  char recvBuf[300];
	  char result[MAX_TOKEN][MAX_LEN];
	  int nCnt;
	  int nIdx = 0;
	  char *beg,*end;
	  int nAddr,nDevIdx;
	  
	  nDevIdx = pLink->value.gpibio.link;
	  nAddr = pLink->value.gpibio.addr;
	  strncpy(recvBuf,&pdpvt->msg[0],pdpvt->msgInputLen);
	  nRet = pdpvt->msgInputLen;
	  
	  recvBuf[nRet] = '\0';
	  
	  if(strncmp(recvBuf,"{p|001",6) != 0)
		  return 0;


	  nCnt = 0;
	  beg = &recvBuf[1];
	  end = &recvBuf[nRet];
	  while(beg<end)
	  {
		  if(*beg != '|')
		  {
			  result[nCnt][nIdx++] = *beg;
		  }
		  else
		  {
			  result[nCnt][nIdx] = '\0';
			  nIdx = 0;
			  ++nCnt;
		  }
		  ++beg;
	  }
	  ++nCnt;

//#define DEBUG_PING1
#ifdef DEBUG_PING1
	  epicsPrintf("source: %s\n",recvBuf);
	  epicsPrintf("mySplit count = %d\n",nCnt);
	  for(nIdx=0;nIdx<nCnt;++nIdx)
		  epicsPrintf("[%d]: %s\n",nIdx,result[nIdx]);
#endif

	SDN[nDevIdx].dbSolonoidPs1VoltRead = atof(result[2]);
	SDN[nDevIdx].dbSolonoidPs1CurrRead = atof(result[3]);
	SDN[nDevIdx].dbSolonoidPs1CurrSet = atof(result[4]);
	SDN[nDevIdx].dbSolonoidPs2VoltRead = atof(result[5]);
	SDN[nDevIdx].dbSolonoidPs2CurrRead = atof(result[6]);
	SDN[nDevIdx].dbSolonoidPs3VoltRead = atof(result[7]);
	SDN[nDevIdx].dbSolonoidPs3CurrRead = atof(result[8]);
	SDN[nDevIdx].dbSolonoidPs4VoltRead = atof(result[9]);
	SDN[nDevIdx].dbPresRead1 = atof(result[10]);

	  pBi->val = 1;
	  return 0;
}

static int procPing2Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3)
{
	  asynUser *pasynUser = pdpvt->pasynUser;
	  struct biRecord *pBi= (struct biRecord *)pdpvt->precord;
	  struct link *pLink = (struct link *)&pBi->inp;
	  asynOctet *pasynOctet = pdpvt->pasynOctet;
	  void *asynOctetPvt = pdpvt->asynOctetPvt;

	  int j;
	  size_t nRet;
	  char recvBuf[300];
	  char result[MAX_TOKEN][MAX_LEN];
	  int nCnt;
	  int nIdx = 0;
	  char *beg,*end;
	  int nAddr,nDevIdx;
	  
	  nDevIdx = pLink->value.gpibio.link;
	  nAddr = pLink->value.gpibio.addr;
	  strncpy(recvBuf,&pdpvt->msg[0],pdpvt->msgInputLen);
	  nRet = pdpvt->msgInputLen;
	  
	  recvBuf[nRet] = '\0';
	  
	  if(strncmp(recvBuf,"{p|002",6) != 0)
		  return 0;
	  
	  nCnt = 0;
	  beg = &recvBuf[1];
	  end = &recvBuf[nRet];
	  while(beg<end)
	  {
		  if(*beg != '|')
		  {
			  result[nCnt][nIdx++] = *beg;
		  }
		  else
		  {
			  result[nCnt][nIdx] = '\0';
			  nIdx = 0;
			  ++nCnt;
		  }
		  ++beg;
	  }
	  ++nCnt;

//#define DEBUG_PING2
#ifdef DEBUG_PING2
	  epicsPrintf("source: %s\n",recvBuf);
	  epicsPrintf("mySplit count = %d\n",nCnt);
	  for(nIdx=0;nIdx<nCnt;++nIdx)
		  epicsPrintf("[%d]: %s\n",nIdx,result[nIdx]);
#endif

	SDN[nDevIdx].dbStandByCurrSet = atof(result[2]);
	SDN[nDevIdx].dbSolonoidPs2CurrSet = atof(result[6]);
	SDN[nDevIdx].dbControlWordSet = strtoul(result[3],NULL,16);
	SDN[nDevIdx].dbSolonoidPs3CurrSet = atof(result[7]);
	SDN[nDevIdx].dbSolonoidPs4CurrSet = atof(result[8]);
	SDN[nDevIdx].dbSolonoidPs2CurrHighLimit = atof(result[4]);			// tunnel vacuum1 high limit
	SDN[nDevIdx].dbSolonoidPs2CurrLowLimit = atof(result[5]);			// tunnel vacuum1 low limit

	  pBi->val = 1;
	  return 0;
}

static int procPing3Msg(struct gpibDpvt *pdpvt, int P1, int P2, char **P3)
{
	  asynUser *pasynUser = pdpvt->pasynUser;
	  struct biRecord *pBi= (struct biRecord *)pdpvt->precord;
	  struct link *pLink = (struct link *)&pBi->inp;
	  asynOctet *pasynOctet = pdpvt->pasynOctet;
	  void *asynOctetPvt = pdpvt->asynOctetPvt;

	  int j;
	  size_t nRet;
	  char recvBuf[300];
	  char result[MAX_TOKEN][MAX_LEN];
	  int nCnt;
	  int nIdx = 0;
	  char *beg,*end;
	  int nAddr,nDevIdx;
	  
	  nDevIdx = pLink->value.gpibio.link;
	  nAddr = pLink->value.gpibio.addr;
	  strncpy(recvBuf,&pdpvt->msg[0],pdpvt->msgInputLen);
	  nRet = pdpvt->msgInputLen;
	  
	  recvBuf[nRet] = '\0';
	  
	  if(strncmp(recvBuf,"{p|003",6) != 0)
		  return 0;
	  
	  nCnt = 0;
	  beg = &recvBuf[1];
	  end = &recvBuf[nRet];
	  while(beg<end)
	  {
		  if(*beg != '|')
		  {
			  result[nCnt][nIdx++] = *beg;
		  }
		  else
		  {
			  result[nCnt][nIdx] = '\0';
			  nIdx = 0;
			  ++nCnt;
		  }
		  ++beg;
	  }
	  ++nCnt;

//#define DEBUG_PING3
#ifdef DEBUG_PING3
	  epicsPrintf("source: %s\n",recvBuf);
	  epicsPrintf("mySplit count = %d\n",nCnt);
	  for(nIdx=0;nIdx<nCnt;++nIdx)
		  epicsPrintf("[%d]: %s\n",nIdx,result[nIdx]);
#endif

	  pBi->val = 1;
	  return 0;
}
static int convertAoData(struct gpibDpvt *pdpvt, int P1, int P2, char **P3)
{
	asynUser *pasynUser = pdpvt->pasynUser;
	struct aoRecord *pAo = (struct aoRecord *)pdpvt->precord;
	struct link *pLink = (struct link *)&pAo->out;
	asynOctet *pasynOctet = pdpvt->pasynOctet;
	void *asynOctetPvt = pdpvt->asynOctetPvt;
	
	unsigned char pact = pAo->pact;
	
	int nAddr,nDevIdx;
	int nNum;
	long lStatus = 0;
	double dbVal;
		
	nDevIdx = pLink->value.gpibio.link;
	nAddr = pLink->value.gpibio.addr;
	sscanf(pLink->value.gpibio.parm,"%d",&nNum);
	
	if(!pact && pAo->pact)
		return 0;

	pAo->pact = TRUE;
	switch(nNum)
	{
		case 60:	SDN[nDevIdx].SADI[nAddr].bUse = (int)pAo->val; break;
		case 61:	SDN[nDevIdx].SADI[nAddr].dbTripHighLimit = pAo->val; break;
		case 62:	SDN[nDevIdx].SADI[nAddr].dbAlarmHighLimit = pAo->val; break;
		case 63:	SDN[nDevIdx].SADI[nAddr].dbAlarmLowLimit = pAo->val; break;
		case 64:	SDN[nDevIdx].SADI[nAddr].dbTripLowLimit = pAo->val; break;
		case 65:	SDN[nDevIdx].SADI[nAddr].dbHVRampSpeed = pAo->val; break;
		case 66:	SDN[nDevIdx].SADI[nAddr].dbHVRampCheckTime = pAo->val; break;		
		case 67:	SDN[nDevIdx].SADI[nAddr].dbHVMaxPoint = pAo->val; break;
		case 68:	SDN[nDevIdx].SADI[nAddr].dbTripBlockingTime = pAo->val; break;
		case 69:	SDN[nDevIdx].SADI[nAddr].dbAlarmBlockingTime = pAo->val; break;
		case 70:	SDN[nDevIdx].SADI[nAddr].dbAlarmDecreaseTime = pAo->val; break;
		case 71:	SDN[nDevIdx].SADI[nAddr].dbHVTripGain = pAo->val; break;
		case 72:	SDN[nDevIdx].SADI[nAddr].dbHVAlarmGain = pAo->val; break;
	}

	pAo->pact = FALSE;

	if(RTN_SUCCESS(lStatus))
		pAo->udf = FALSE;
	return 0;
}

static int convertAiData(struct gpibDpvt *pdpvt, int P1, int P2, char **P3)
{
	asynUser *pasynUser = pdpvt->pasynUser;
	struct aiRecord *pAi = (struct aiRecord *)pdpvt->precord;
	struct link *pLink = (struct link *)&pAi->inp;
	asynOctet *pasynOctet = pdpvt->pasynOctet;
	void *asynOctetPvt = pdpvt->asynOctetPvt;
	
	unsigned char pact = pAi->pact;
	
	int nAddr,nDevIdx;
	int nNum;
	long lStatus = 0;
	double dbVal;
		
	nDevIdx = pLink->value.gpibio.link;
	nAddr = pLink->value.gpibio.addr;
	sscanf(pLink->value.gpibio.parm,"%d",&nNum);
	
	if(!pact && pAi->pact)
		return 0;

	pAi->pact = TRUE;

	switch(nNum){
		case 2:		dbVal = SDN[nDevIdx].dbStateSet; break;
		case 4:		dbVal = SDN[nDevIdx].dbStateRead;	break;
		case 5:		dbVal = SDN[nDevIdx].dbFilamentVoltRead; break;
		case 6:		dbVal = SDN[nDevIdx].dbFilamentCurrRead; break;
		case 7:		dbVal = SDN[nDevIdx].dbCtRead; break;
		case 8:		dbVal = SDN[nDevIdx].dbCvdRead; break;
		case 9:		dbVal = SDN[nDevIdx].dbCtArcPerSecondRead; break;
		case 10:	dbVal = SDN[nDevIdx].dbCvdArcPerSecondRead; break;
		case 11:	dbVal = SDN[nDevIdx].dbPrfRead; break;
		case 12:	dbVal = SDN[nDevIdx].dbPlswthRead; break;
		case 13:	dbVal = SDN[nDevIdx].dbPowRead; break;
		case 14:	dbVal = SDN[nDevIdx].dbHVPSVoltRead; break;
		//ddcase 15:	dbVal = SDN[nAddr].dbHVPSCurrRead; break;
		case 15:	dbVal = SDN[nDevIdx].dbHVPSVoltSet; break;
		case 16:	dbVal = SDN[nDevIdx].dbPlswthSet; break;
		case 17:	dbVal = SDN[nDevIdx].dbPrfSet; break;
		case 18:	dbVal = SDN[nDevIdx].dbRemainingTime; break;
		case 19:	dbVal = SDN[nDevIdx].dbAccessLevel; break;
		case 20:	dbVal = SDN[nDevIdx].dbSolonoidPs1VoltRead; break;
		case 21:	dbVal = SDN[nDevIdx].dbSolonoidPs1CurrRead; break;
		case 22:	dbVal = SDN[nDevIdx].dbSolonoidPs1CurrSet; break;
		case 23:	dbVal = SDN[nDevIdx].dbSolonoidPs2VoltRead; break;
		case 24:	dbVal = SDN[nDevIdx].dbSolonoidPs2CurrRead; break;
		case 25:	dbVal = SDN[nDevIdx].dbSolonoidPs3VoltRead; break;
		case 26:	dbVal = SDN[nDevIdx].dbSolonoidPs3CurrRead; break;
		case 27:	dbVal = SDN[nDevIdx].dbSolonoidPs4VoltRead; break;
		case 28:	dbVal = SDN[nDevIdx].dbPresRead1; break;
		case 31:	dbVal = SDN[nDevIdx].dbStandByCurrSet; break;
		case 32:	dbVal = SDN[nDevIdx].dbSolonoidPs2CurrSet; break;
		case 33:	dbVal = SDN[nDevIdx].dbControlWordSet; break;
		case 34:	dbVal = SDN[nDevIdx].dbSolonoidPs3CurrSet; break;
		case 35:	dbVal = SDN[nDevIdx].dbSolonoidPs4CurrSet; break;
		case 45:	dbVal = SDN[nDevIdx].dbSolonoidPs2CurrHighLimit; break;			// tunnel vacuum1 high limit
		case 46:	dbVal = SDN[nDevIdx].dbSolonoidPs2CurrLowLimit; break;			// tunnel vacuum1 low limit

		case 47:	dbVal = SDN[nDevIdx].SADI[nAddr].bUse; break;
		case 48:	dbVal = SDN[nDevIdx].SADI[nAddr].dbTripHighLimit; break;
		case 49:	dbVal = SDN[nDevIdx].SADI[nAddr].dbAlarmHighLimit; break;
		case 50:	dbVal = SDN[nDevIdx].SADI[nAddr].dbAlarmLowLimit; break;
		case 51:	dbVal = SDN[nDevIdx].SADI[nAddr].dbTripLowLimit; break;
		case 52:	dbVal = SDN[nDevIdx].SADI[nAddr].dbHVRampSpeed; break;
		case 53:	dbVal = SDN[nDevIdx].SADI[nAddr].dbHVRampCheckTime ; break;		
		case 54:	dbVal = SDN[nDevIdx].SADI[nAddr].dbHVMaxPoint; break;
		case 55:	dbVal = SDN[nDevIdx].SADI[nAddr].dbTripBlockingTime; break;
		case 56:	dbVal = SDN[nDevIdx].SADI[nAddr].dbAlarmBlockingTime; break;
		case 57:	dbVal = SDN[nDevIdx].SADI[nAddr].dbAlarmDecreaseTime; break;
		case 58:	dbVal = SDN[nDevIdx].SADI[nAddr].dbHVTripGain; break;
		case 59:	dbVal = SDN[nDevIdx].SADI[nAddr].dbHVAlarmGain; break;
			
	}

	pAi->val = dbVal;

	pAi->pact = FALSE;

	if(RTN_SUCCESS(lStatus))
		pAi->udf = FALSE;

	return 0;
}

static int convertMbbiData(struct gpibDpvt *pdpvt, int P1, int P2, char **P3)
{
	asynUser *pasynUser = pdpvt->pasynUser;
	struct mbbiRecord *pMbbi = (struct mbbiRecord *)pdpvt->precord;
	struct link *pLink = (struct link *)&pMbbi->inp;
	asynOctet *pasynOctet = pdpvt->pasynOctet;
	void *asynOctetPvt = pdpvt->asynOctetPvt;
	
	unsigned char pact = pMbbi->pact;
	
	int nAddr;
	int nNum;
	long lStatus = 0;
	double dbVal;
		
	nAddr = pLink->value.gpibio.addr;
	sscanf(pLink->value.gpibio.parm,"%d",&nNum);
	
	if(!pact && pMbbi->pact)
		return 0;

	pMbbi->pact = TRUE;

	switch(nNum){
		//case 18:	dbVal = SDN[nAddr].dbRemainingTime; break;
	}

	pMbbi->val = (int)dbVal;

	pMbbi->pact = FALSE;

	if(RTN_SUCCESS(lStatus))
		pMbbi->udf = FALSE;

	return 0;
}

static void runAutoDriveThreadFunc(void *lParam)
{
	int i;
	double dbTemp;
	SCANDINOVA_AUTO_DRIVE_INFO *p = (SCANDINOVA_AUTO_DRIVE_INFO*)lParam;

	epicsThreadSleep(5);

	while(1)
	{
		if(p->bUse == 0)
		{
			epicsThreadSleep(1);
			continue;
		}

		// hardware interlock reset
		if((int)SDN[p->nParentId].dbStateRead != 0xD000)
		{
			epicsThreadSleep(p->dbTripBlockingTime);
			hwControlSet(p->nParentId,0x0001);
			epicsThreadSleep(5);
			if((int)SDN[p->nParentId].dbStateRead == 0x06000)
			{
				setHv(p->nParentId,300.0);
				epicsThreadSleep(5);
				changeMode(p->nParentId,0x0D000);
			}
		}

		//epicsPrintf("\n##### vacuum: %.2f\n\n",*p->dbVacuum);

		if(*p->dbVacuum>=p->dbTripHighLimit)			// no.3 section (trip high limit)
		{
			p->bOnArcing = 1;
			p->bOnMidPoint = 1;
			p->dbMidPoint = SDN[p->nParentId].dbHVPSVoltRead * p->dbHVTripGain / 100.0;
			changeMode(p->nParentId, 0x0A000);
			//epicsPrintf("[SCANDINOVA AUTODRIVE] Trip High limit\n");
		}
		else if(*p->dbVacuum >= p->dbAlarmHighLimit)	// no.1 section (s/w alarm high limit)
		{
			dbTemp = SDN[p->nParentId].dbHVPSVoltRead * p->dbHVAlarmGain / 100.0;
			setHv(p->nParentId, dbTemp);
			//epicsPrintf("alarmdecresetime sleep: %.2f\n",p->dbAlarmDecreaseTime);
			epicsThreadSleep(p->dbAlarmDecreaseTime);	// (unit: sec)
			p->bOnAlarm = 1;
			//epicsPrintf("[SCANDINOVA AUTODRIVE] alarm High limit\n");
			//epicsPrintf("volt: %.2f\tgain: %.2f\t:setpoint: %.2f\n",SDN[p->nParentId].dbHVPSVoltRead,p->dbHVAlarmGain,dbTemp);
		}
		else if(*p->dbVacuum >= p->dbAlarmLowLimit)		// normal section
		{
			if(p->bOnArcing == 0 && p->bOnAlarm == 0)
				increaseHv(p);	// 
			//epicsPrintf("[SCANDINOVA AUTODRIVE] normal\n");
		}
		else if(*p->dbVacuum >= p->dbTripLowLimit)		// no.2 section (s/w alarm low limit)
		{
			//epicsPrintf("[SCANDINOVA AUTODRIVE] alarm low limit\n");
			if(p->bOnAlarm == 1)
			{
				//epicsPrintf("alarmblockingtime sleep: %.2f\n",p->dbAlarmBlockingTime);
				epicsThreadSleep(p->dbAlarmBlockingTime);
				p->bOnAlarm = 0;
			}
			if(p->bOnAlarm == 0 && p->bOnArcing == 0)
				increaseHv(p);
		}
		else											// no.4 section (h/w trip low limit)
		{
			//epicsPrintf("[SCANDINOVA AUTODRIVE] trip low limit\n");
			if(p->bOnArcing == 1)
			{
				p->bOnArcing = 0;
				changeMode(p->nParentId, 0x0d000);
				increaseHv(p);
			}
			else
				increaseHv(p);
		}

		//epicsPrintf("Thread... [%d][%d] : %.2f - %.2f - %.2f\n",p->nParentId,p->nIdx
		//		,p->dbTripHighLimit,*p->dbVacuum,p->dbTripLowLimit);
		epicsThreadSleep(1);
	}
}

int changeMode(int nDevIdx, int nMode)
{
	char strCmd[256];
	sprintf(strCmd,"dbpf FEL:ATF03:LO_STATE_SET %d",nMode);
	iocshCmd(strCmd);

	//epicsPrintf("state change. %X\n",nMode);
	return 1;
}
int setHv(int nDevIdx, double dbSetpoint)
{ 
	char strCmd[256];
	sprintf(strCmd,"dbpf FEL:ATF03:AO_HVPS_VOLT_SETPOINT %.2f",dbSetpoint);
	iocshCmd(strCmd);
	
	//epicsPrintf("setpoint change. %.2f\n",dbSetpoint);
	return 1;
}
int hwControlSet(int nDevIdx, int nMode)
{
	char strCmd[256];
	sprintf(strCmd,"dbpf FEL:ATF03:MBO_CONTROL_WORD_SET %d",nMode);
	iocshCmd(strCmd);
	
	//epicsPrintf("setpoint change. %.2f\n",dbSetpoint);
	return 1;
}
int increaseHv(SCANDINOVA_AUTO_DRIVE_INFO *p)
{
	double dbOffset;
	double dbSub;
	char strCmd[256];
	epicsTimeStamp tNow;
	
	// only master
	if(p->nPriority == SLAVE)
		return 1;

	dbOffset = 2;

	// time check 
	epicsTimeGetCurrent(&tNow);
	dbSub = epicsTimeDiffInSeconds(&tNow,&p->tLastIncrease);
	if(SDN[p->nParentId].dbHVPSVoltRead <= 1000)
	{
		if(p->bOnMidPoint == 1)
		{
			if(dbSub >= 10.0
					&& SDN[p->nParentId].dbHVPSVoltRead >= SDN[p->nParentId].dbHVPSVoltSet-dbOffset
					&& SDN[p->nParentId].dbHVPSVoltRead < p->dbMidPoint-dbOffset)
			{
				sprintf(strCmd,"dbpf FEL:ATF03:AO_HVPS_VOLT_SETPOINT %.2f",fmin(SDN[p->nParentId].dbHVPSVoltSet + 10.0,p->dbHVMaxPoint));
				iocshCmd(strCmd);
				epicsTimeGetCurrent(&p->tLastIncrease);
				//epicsPrintf("increase hv : %.2f\n",fmin(SDN[p->nParentId].dbHVPSVoltSet + p->dbHVRampSpeed,p->dbHVMaxPoint));
			}
			else
			{
				//epicsPrintf("tripblockingtime sleep: %.2f\n",p->dbTripBlockingTime);
				epicsThreadSleep(p->dbTripBlockingTime);
				p->bOnMidPoint = 0;
			}
		}
		else
		{
			if(dbSub >= 10.0
					&& SDN[p->nParentId].dbHVPSVoltRead >= SDN[p->nParentId].dbHVPSVoltSet-dbOffset
					&& SDN[p->nParentId].dbHVPSVoltRead < p->dbHVMaxPoint-dbOffset)
			{
				sprintf(strCmd,"dbpf FEL:ATF03:AO_HVPS_VOLT_SETPOINT %.2f",fmin(SDN[p->nParentId].dbHVPSVoltSet + 10.0,p->dbHVMaxPoint));
				iocshCmd(strCmd);
				epicsTimeGetCurrent(&p->tLastIncrease);
				//epicsPrintf("increase hv : %.2f\n",fmin(SDN[p->nParentId].dbHVPSVoltSet + p->dbHVRampSpeed,p->dbHVMaxPoint));
			}
			else
			{
				//epicsPrintf("increaseHv failure.\n");
				//epicsPrintf("dbSub: %f\n",dbSub);
				//epicsPrintf("dbHVRampSpeed: %f\n", p->dbHVRampSpeed);
				//epicsPrintf("dbHVPSVoltRead: %f\n",SDN[p->nParentId].dbHVPSVoltRead);
				//epicsPrintf("dbHVPSVoltSet: %f\n",SDN[p->nParentId].dbHVPSVoltSet);
				//epicsPrintf("dbHVMaxPoint: %f\n",p->dbHVMaxPoint);
			}
		}
	}
	else
	{
		if(p->bOnMidPoint == 1)
		{
			if(dbSub >= p->dbHVRampCheckTime
					&& SDN[p->nParentId].dbHVPSVoltRead >= SDN[p->nParentId].dbHVPSVoltSet-dbOffset
					&& SDN[p->nParentId].dbHVPSVoltRead < p->dbMidPoint-dbOffset)
			{
				sprintf(strCmd,"dbpf FEL:ATF03:AO_HVPS_VOLT_SETPOINT %.2f",fmin(SDN[p->nParentId].dbHVPSVoltSet + p->dbHVRampSpeed,p->dbHVMaxPoint));
				iocshCmd(strCmd);
				epicsTimeGetCurrent(&p->tLastIncrease);
				//epicsPrintf("increase hv : %.2f\n",fmin(SDN[p->nParentId].dbHVPSVoltSet + p->dbHVRampSpeed,p->dbHVMaxPoint));
			}
			else
			{
				//epicsPrintf("tripblockingtime sleep: %.2f\n",p->dbTripBlockingTime);
				epicsThreadSleep(p->dbTripBlockingTime);
				p->bOnMidPoint = 0;
			}
		}
		else
		{
			if(dbSub >= p->dbHVRampCheckTime
					&& SDN[p->nParentId].dbHVPSVoltRead >= SDN[p->nParentId].dbHVPSVoltSet-dbOffset
					&& SDN[p->nParentId].dbHVPSVoltRead < p->dbHVMaxPoint-dbOffset)
			{
				sprintf(strCmd,"dbpf FEL:ATF03:AO_HVPS_VOLT_SETPOINT %.2f",fmin(SDN[p->nParentId].dbHVPSVoltSet + p->dbHVRampSpeed,p->dbHVMaxPoint));
				iocshCmd(strCmd);
				epicsTimeGetCurrent(&p->tLastIncrease);
				//epicsPrintf("increase hv : %.2f\n",fmin(SDN[p->nParentId].dbHVPSVoltSet + p->dbHVRampSpeed,p->dbHVMaxPoint));
			}
			else
			{
				//epicsPrintf("increaseHv failure.\n");
				//epicsPrintf("dbSub: %f\n",dbSub);
				//epicsPrintf("dbHVRampSpeed: %f\n", p->dbHVRampSpeed);
				//epicsPrintf("dbHVPSVoltRead: %f\n",SDN[p->nParentId].dbHVPSVoltRead);
				//epicsPrintf("dbHVPSVoltSet: %f\n",SDN[p->nParentId].dbHVPSVoltSet);
				//epicsPrintf("dbHVMaxPoint: %f\n",p->dbHVMaxPoint);
			}
		}
	}

	return 1;
}
