#ifndef __EC_MASTER_API_H
#define __EC_MASTER_API_H

#include <vector>

#if (defined SYLIXOS)
#include <stdlib.h>
#endif

#define EC_FALSE            0
#define EC_TRUE             1
#define EC_NULL             0

#define EC_T_VOID           void

typedef void*           EC_T_PVOID;

typedef int             EC_T_BOOL;

typedef char            EC_T_CHAR;
typedef unsigned short  EC_T_WCHAR;

typedef unsigned char   EC_T_BYTE, *EC_T_PBYTE;
typedef unsigned short  EC_T_WORD;
typedef unsigned int    EC_T_DWORD;

typedef signed char	    EC_T_SBYTE;
typedef signed short    EC_T_SWORD;
typedef signed int      EC_T_SDWORD;

typedef int             EC_T_INT;
typedef unsigned int    EC_T_UINT;

typedef short           EC_T_SHORT;
typedef unsigned short  EC_T_USHORT;

typedef long long int           EC_T_LONG;
typedef unsigned long long int  EC_T_ULONG;

typedef double			EC_T_REAL;

typedef void*			EC_T_ADDRESS;

typedef void*			EC_T_HANDLE;

struct CycleTime
{
    EC_T_DWORD      curr;             /* [usec] */
    EC_T_DWORD      min;              /* [usec] */
    EC_T_DWORD      max;              /* [usec] */
    EC_T_DWORD      avg;              /* [usec] */
    CycleTime ()
    {
        curr = 0;
        min = 0;
        max = 0;
        avg = 0;
    };
};

typedef void (*EC_PF_EC_START_Finish_CALLBACK)(int count);
typedef int (*EC_PF_EC_START_ContrastID_CALLBACK)(const std::vector < std::pair<unsigned int, unsigned int> > &);
typedef int (*EC_PF_EC_START_AppWorkpd_CALLBACK)();
typedef void (*EC_PF_EC_START_CustomeLog_CALLBACK)(unsigned char , const char *, const char *, const char *, const long , const char *, ...);

void registerCustomeContrastID(EC_PF_EC_START_ContrastID_CALLBACK p);
void registerCustomeAppWorkpd(EC_PF_EC_START_AppWorkpd_CALLBACK p);
void registerCustomeEcMasterStartFinish(EC_PF_EC_START_Finish_CALLBACK p);
void registerCustomeAppLog(EC_PF_EC_START_CustomeLog_CALLBACK p);

extern int startEcMaster(int nArgc, char* ppArgv[]);
extern EC_T_DWORD enableRealtimeEnvironment();
extern void setSignalHandler();

extern EC_T_DWORD ECM_LogMsg(const EC_T_CHAR* szFormat, ...);
extern EC_T_DWORD ECM_LogError(const EC_T_CHAR* szFormat, ...);

extern EC_T_DWORD ecatGetConnectedSlavesNum();
extern EC_T_DWORD ecatGetConfiguredSlavesNum();
extern EC_T_WORD ecatGetSlaveState(EC_T_DWORD ecSlaveNum);//ecSlaveNum从0开始

extern int ecatSetSDO(EC_T_DWORD slaveNum, EC_T_WORD index, EC_T_DWORD subindex, EC_T_BYTE* value, EC_T_DWORD size);//slaveNum从0开始
extern int ecatGetSDO(EC_T_DWORD slaveNum, EC_T_WORD index, EC_T_DWORD subindex, EC_T_BYTE* value, EC_T_DWORD size, int num = 0);//slaveNum从0开始

extern void setEcatLicenseKey(const char* key);
extern bool isEcatLicenseCorrect();
extern void setLogDirName(const char* dir);
extern void setEcatLogSwitch(bool key);

extern CycleTime getCycleTime();


typedef struct
{
	EC_T_WORD index;
	EC_T_WORD subIndex;
	EC_T_BYTE* addrMap;
} PDOAddrData;
typedef std::vector<PDOAddrData> PDOAddrOneVec;
typedef std::vector<PDOAddrOneVec> PDOAddrVec;
extern void getPDOAddrVec(PDOAddrVec& out, PDOAddrVec& in);

extern void getSlaveIDVec(std::vector<std::pair<unsigned int, unsigned int> > &IDVec);

#ifdef __cplusplus
extern "C"
{
#endif

typedef EC_T_VOID (*EC_PF_THREADENTRY)(EC_T_VOID* pvParams);
extern EC_T_VOID*  OsCreateThread(EC_T_CHAR* szThreadName, EC_PF_THREADENTRY pfThreadEntry, EC_T_DWORD dwPrio, EC_T_DWORD dwStackSize, EC_T_VOID* pvParams);

EC_T_VOID LinuxSleep(EC_T_DWORD dwMsec);

#ifdef __cplusplus
} /* extern "C"*/
#endif

#if (defined LINUX) && !(defined XENOMAI)
#define OsSleep(dwMsec)                LinuxSleep(dwMsec)
#endif

#ifndef OsSleep
extern EC_T_VOID   OsSleep(EC_T_DWORD dwMsec);
#endif

#ifndef OsPrintf
#define OsPrintf                                printf
#endif

#if (defined SYLIXOS) || (USE_IGH_EC_LIB)
struct workPd{
	uint8_t  *(*processPd);            /*  配置的域数据                */
	uint32_t *uiRxPdoNum;
	uint32_t *uiRxLen;
	uint32_t *uiTxPdoNum;
	uint32_t *uiTxLen;
};

int ecatGetPdo();
#endif

#endif /* __EC_MASTER_API_H */
