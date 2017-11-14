#include "hdr.h"

#undef TEST_MODE
//#define TEST_MODE

//******************************************
#ifdef	x86
    #define max_card	8
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <stdbool.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <pthread.h>
#include <string.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <openssl/ssl.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <syslog.h>
#include <asm/termbits.h>
#else
    #define max_card	6
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <stdbool.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <pthread.h>
#include <string.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <openssl/ssl.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <syslog.h>
#include <asm/termbits.h>
#endif


#define kol_card		max_card

#ifdef x86
    #define all_radio	32
#else
    #define all_radio	24

#endif

#define CS_PiML_AT_COM		0x1010 //  базовый адрес управления COM-портами AT (данные)
#define CS_PRESENCE		0x11E0 //  адрес идентификатора плат
#define CS_ROM   		0x11D0 //  адрес идентификатора прошивки на субплате
#define AdrModuleStatus 	0x1000 //  базовый адрес управления COM-портами2 (статус) IMEI
#define AdrModuleDataIMEI 	0x1030 //  базовый адрес управления COM-портами2 запись  IMEI
#define CS_RESET   		0x11F0 //  базовый адрес сброса плат
#define CS_PiML_STATUS		0x1000 //  базовый адрес управления/чтения статуса модулей
//read : 7 6 5 4 3 2 1 0	write : 7 6 5 4 3 2 1 0
//	 W R R W R W R V		\-/ \S/ M R O P
//       R D S R D R D I                 S   P  O E N W
//           T         O                 P   E  D S / R
//       I I   S S E E                   E   E  E E O
//       M M S I I M M                   E   D    T F
//       E E I M M P P                   D   2    P F
//       I I M                           1        i
//						  M
//						  L
//				     SPEED1:00-9600, 10-28800, 11-115200 (для CS_PiML_AT_COM)
//
//				                MODE:0-обычный режим приемо-предатчика АТ команд модуля
//						     1-программирование(смена прошивки) модуля
//
//						  SPEED2: 00-9600,
//							  01-55200
//							  10-102000 (для CS_PiML_SIM_COM)
//
//						    ON/OFF вкл./выкл. модуля
//
//						      PWR - вкл./выкл. питания модуля

#define dln_32 32
#define dln_64 64
#define dln_80 80
#define dln_128 128
#define dln_256 256
#define dln_384 384
#define dln_512 512
#define dln_1024 1024
#define dln_2048 2048
#define dln_4096 4096

#define VIO    1            //  бит статуса "модуль вкл.выкл"
#define RDEMP  2            //  бит статуса "буфер для чтения COM-порта готов"
#define WREMP  4            //  бит статуса "буфер для записи в COM-порт готов"
#define MOD_RDempty  8      //  бит статуса "буфер для чтения COM-порта SIM готов"
#define MOD_WRempty  0x10   //  бит статуса "буфер для записи в COM-порт SIM готов"
#define MOD_RES 0x20        //  сигнал сброса, поступающий от модуля
#define CTS 0x40            //  сигнал CTS модема

#define ModPwrOnOff 1	    //  управление вкл.выкл питания модуля
#define ModOnOff    2       //  управление вкл.выкл модуля (конт. ON/OFF)
#define ModRstOnOff 4       //  сигнал сброса модуля (только для PiML)
#define ModeATBuff  8       //  режим синхронизации (0 - старый ражим с синхр. по 0x0d,0x0a, 
			    //  1 - новый режим без синхронизации по 0x0d,0x0a )
#define ReciveCME     0x01  // принят +CME
#define ReciveERR     0x02  // принят ERROR
#define ReciveOK      0x04  // принят ОК
#define ReciveCom     0x08  // принят ответ на команду

#define _10ms		1
#define _20ms		2
#define _30ms		3
#define _40ms		4
#define _50ms		5
#define _60ms		6
#define _70ms		7
#define _80ms		8
#define _90ms		9
#define _100ms		10
#define _110ms		11
#define _120ms		12
#define _130ms		13
#define _140ms		14
#define _150ms		15
#define _160ms		16
#define _170ms		17
#define _180ms		18
#define _190ms		19
#define _200ms		20
#define _210ms		21
#define _220ms		22
#define _230ms		23
#define _240ms		24
#define _250ms		25
#define _260ms		26
#define _270ms		27
#define _280ms		28
#define _290ms		29
#define _300ms		30
#define _350ms		35
#define _360ms		36
#define _370ms		37
#define _380ms		38
#define _390ms		39
#define _400ms		40
#define _450ms		45
#define _500ms		50
#define _550ms		55
#define _600ms		60
#define _650ms		65
#define _700ms		70
#define _750ms		75
#define _800ms		80
#define _850ms		85
#define _900ms		90
#define _950ms		95
#define _1s		100
#define _1_5s		150
#define _1_6s		160
#define _1_7s		170
#define _1_8s		180
#define _1_9s		190
#define _2s		200
#define _2_1s		210
#define _2_2s		220
#define _2_3s		230
#define _2_5s		250
#define _3s		300
#define _3_5s		350
#define _4s		400
#define _4_5s		450
#define _5s		500
#define _6s		600
#define _7s		700
#define _8s		800
#define _9s		900
#define _10s		1000
#define _11s		1100
#define _12s		1200
#define _13s		1300
#define _14s		1400
#define _15s		1500
#define _16s		1600
#define _17s		1700
#define _18s		1800
#define _19s		1900
#define _20s		2000
#define _25s		2500
#define _30s		3000
#define _31s		3100
#define _32s		3200
#define _33s		3300
#define _34s		3400
#define _35s		3500
#define _36s		3600
#define _37s		3700
#define _38s		3800
#define _39s		3900
#define _40s		4000
#define _43s		4300
#define _45s		4500
#define _50s		5000
#define _55s		5500
#define _58s		5800
#define _60s		6000
#define _70s		7000
#define _75s		7500
#define _80s		8000
#define _90s		9000
#define _100s		10000
#define _110s		11000
#define _2m		12000
#define _2_5m		15000
#define _3m		18000
#define _5m		6000*5
#define _10m		6000*10
#define _12m		6000*12
#define _15m		6000*15
#define _30m		6000*30
#define _60m		6000*60


#define max_at 4
#define max_buf_len 1024
#define max_err 2
#define max_status 6
#define max_data_len 1008

#define VERSION			0x00000001
#define MARKER			0xAA
#define CMD_DL_BEGIN		0x0001
#define CMD_DL_BEGIN_RESP	0x0002
#define CMD_DL_DATA		0x0003
#define CMD_DL_DATA_RESP	0x0004
#define CMD_DL_END		0x0005
#define CMD_DL_END_RESP		0x0006
#define CMD_RUN_GSMSW		0x0007
#define CMD_RUN_GSMSW_RESP	0x0008

//******************************************************************************
//******************************************************************************
//******************************************************************************
//				TYPES
#pragma pack(push,1)
typedef struct
{
unsigned char StatusAdp; //статус СОМ-порта радиоканала
unsigned char MainError; //код ошибки СОМ-порта радиоканала
unsigned char TX_com[max_buf_len+1]; //буфер предаваемой команды модулю радиоканала
unsigned char RX_com[max_buf_len+1]; //буфер принимаемой команды от модуля радиоканала
unsigned short tx_numb; //количество переданных символов команды 
unsigned short rx_numb; //количество принятых символов команды 
unsigned char tx_faza; //фаза обработки
unsigned char rx_faza; //фаза обработки
unsigned short Count_Error; //счетчик ошибок
unsigned char rx_ready; //признак полного пакета 1-полный 0-нет
unsigned char faza;
} s_COM;
//---------------------------
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned int data;
unsigned short crc16;
} s_dl_begin;
//---------------------------
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned int seq_num;
//unsigned char *data;
//unsigned short crc16;
} s_dl_data;
//---------------------------
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short crc16;
} s_dl_end;
//---------------------------
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short crc16;
} s_run_gsmsw;
//---------------------------
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short status;
} s_resp;
//---------------------------
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short status;
unsigned short crc16;
} s_ack;
//---------------------------
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short status;
unsigned int seq_num;
unsigned short crc16;
} s_ack_data;
#pragma pack(pop)

//******************************************************************************
//******************************************************************************
//******************************************************************************
//				VAR

extern s_ack hdr_resp;

extern int MaxLogLevel;

extern int ErrorSignal;

extern unsigned char dmp;
extern unsigned char SIGHUPs;
extern unsigned char SIGTERMs;
extern unsigned char SIGINTs;
extern unsigned char SIGSEGVs;

extern const char *vio_up;
extern const char *vio_down;
extern char const * fillog;
extern int loops, fd, fd_timer;
extern unsigned int tmr_ms, tmr_rx, tmr_tx, tmr0;
extern unsigned long long tmr_one;

extern const char *cmd[];
extern const char *msg_status[max_status];

extern unsigned short RK_index;
extern unsigned char RK_Mir, at_data, Def_RK;

extern s_COM COM;

extern unsigned char faza,last_faza;

extern char txd[max_buf_len];
extern char rxd[max_buf_len];
extern char module[max_buf_len];
extern char module_new[max_buf_len];
extern char *uk_rxd;


extern unsigned char vio,last_vio;
extern unsigned char on_off;//1-on  0-off
extern unsigned char pwr_on_off;//1-on  0-off

extern unsigned int seq_number, seq_number_resp, pk_all;
extern int pk_now, pk_len, pk_status, file_size, last_size;
extern unsigned short pk_max_dl;

extern unsigned char SYNC_WORD[2];
extern unsigned char SYNC_WORD_RESP[2];

extern unsigned char *bin;
extern int fsize, fd_bin;
extern char fname[256];

extern char labka[128];

// ************************************************************************

extern unsigned int get_timer(unsigned int t);
extern int check_delay(unsigned int t);
extern char *CurTime();
extern char * DTNowPrn4(char *lab);
extern void LogMsg(int LogLevel, const char * const Msg);
extern void PrintErrorSignal(int es);
extern int print_msg(const char *st);
extern void string_AT_com(const char *st);
extern void _ReadSig(int sig);
extern void _TermSig(int sig);
extern void _IntSig(int sig);
extern void _SegvSig(int sig);
extern void my_CloseAll();
extern void ind_rk(unsigned char nk);
extern void SayToCore(unsigned char nk);
extern void print_faza(unsigned char sk);
extern void ModulePWR_ON(unsigned char nk);
extern void ModulePWR_OFF(unsigned char nk);
extern void Module_ON(unsigned char nk);
extern void Module_OFF(unsigned char nk);
extern unsigned char ChangeSpeed(unsigned char nk);
extern void reset_timer_tmr(unsigned char tp);//tmr_mod <- clear long_long
extern int check_timer_tmr(unsigned long long t);//tmr_mod
extern unsigned long long get_timer_tmr(unsigned long long t);//tmr_mod
extern unsigned char myhextobin(char st, char ml);
extern unsigned char check_vio(unsigned char nk);
extern void put_AT_com(const char *st);
extern void put_AT_data(unsigned char *dat, unsigned short len);
extern int RXdone(unsigned char nk);
extern unsigned char RXbyte_prn(unsigned char nk, unsigned char prn_bt, unsigned char scr);
extern int RXbyte(unsigned char nk, unsigned char scr);
extern int RXbytes22(unsigned char nk, char * outs, unsigned char at);
extern int TXdone(unsigned char nk);
extern int TXdone2(unsigned char nk);
extern int TXbyte(unsigned char nk, unsigned char bb, unsigned char fl);
extern int TXbyte_all(unsigned char nk, unsigned char * st, unsigned short len);
extern unsigned char TXCOM2_all(unsigned char nk, unsigned char at);
extern void SetTickMode(unsigned char tmode);
extern unsigned short mk_crc16(unsigned char *data, unsigned short len);
extern int mk_dl_begin(unsigned char prn);
extern int mk_dl_end(unsigned char prn);
extern int mk_run_gsmsw(unsigned char prn);
extern int mk_dl_data(unsigned char prn, int len, unsigned int s_n);
extern int parse_answer(unsigned char prn, unsigned char *data, int len,  unsigned int *s_n, unsigned short *m_d);
extern int ReadBIN(char *fn);
extern void ClearRX();
extern int mk_ack_data(unsigned int s_n, unsigned char *out);
extern void SetKT(unsigned char nk);

