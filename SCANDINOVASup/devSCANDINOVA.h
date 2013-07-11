#ifndef DEVSCANDINOVA_H
#define DEVSCANDINOVA_H

#define MAX_SCANDINOVA_CNT				1
#define MAX_SCANDINOVA_VACUUM_COUNT		6

#define MASTER							1
#define SLAVE							0

#include <epicsTime.h>

typedef struct
{
	epicsTimeStamp tLastIncrease;
	int nPriority;
	int nParentId;
	int bUse;
	int nIdx;

	int bOnArcing;
	int bOnAlarm;
	int bOnMidPoint;
	double dbMidPoint;
	double dbHVMaxPoint;

	double *dbVacuum;
	double dbTripHighLimit;
	double dbAlarmHighLimit;
	double dbAlarmLowLimit;
	double dbTripLowLimit;

	double dbHVTripGain;			// gain(unit: %)(ex. 90%)
	double dbHVAlarmGain;		// gain(unit: %)(ex. 90%)
	double dbTripBlockingTime;		// (unit: sec)
	double dbAlarmBlockingTime;		// (unit: sec)
	double dbAlarmDecreaseTime;		// (unit: sec)

	double dbHVRampSpeed;			// rising time(dbHVRampSpeed / dbHVRampCheckTime) (v/sec)
	double dbHVRampCheckTime;		
} SCANDINOVA_AUTO_DRIVE_INFO;

typedef struct 
{
	int nDevIdx;
	// ping 0
	double dbStateSet;
	double dbStateRead;
	double dbFilamentVoltRead;
	double dbFilamentCurrRead;
	double dbCtRead;
	double dbCvdRead;
	double dbCtArcPerSecondRead;
	double dbCvdArcPerSecondRead;
	double dbPrfRead;
	double dbPlswthRead;
	double dbPowRead;
	double dbHVPSVoltRead;
	//double dbHVPSCurrRead; // none
	double dbHVPSVoltSet;
	double dbPlswthSet;
	double dbPrfSet;
	double dbRemainingTime;
	double dbAccessLevel;
	 
	// ping 1
	double dbSolonoidPs1VoltRead;
	double dbSolonoidPs1CurrRead;
	double dbSolonoidPs1CurrSet;
	double dbSolonoidPs2VoltRead;
	double dbSolonoidPs2CurrRead;
	double dbSolonoidPs3VoltRead;
	double dbSolonoidPs3CurrRead;
	double dbSolonoidPs4VoltRead;
	double dbPresRead1;

	// ping 2
	double dbStandByCurrSet;
	double dbSolonoidPs2CurrSet;
	double dbControlWordSet;
	double dbSolonoidPs3CurrSet;
	double dbSolonoidPs4CurrSet;
	double dbSolonoidPs2CurrHighLimit;
	double dbSolonoidPs2CurrLowLimit;
	
	// ping 3
	

	// auto drive
	SCANDINOVA_AUTO_DRIVE_INFO SADI[MAX_SCANDINOVA_VACUUM_COUNT];
	//double dbHVPSVoltRead;
	//double dbHVPSVoltSet;
	//double dbVacHVSet;		// max hvps value (ex. 1290.0v)


} SCANDINOVA_INFO;

extern SCANDINOVA_INFO SDN[MAX_SCANDINOVA_CNT];

extern SCANDINOVA_AUTO_DRIVE_INFO SADI[MAX_SCANDINOVA_VACUUM_COUNT];

#endif
