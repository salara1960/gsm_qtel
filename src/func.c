#include "func.h"

// ************************************************************************

s_ack hdr_resp;

int MaxLogLevel = LOG_DEBUG + 1;
int ErrorSignal = 0;

unsigned char dmp = 0;
unsigned char SIGHUPs = 1;
unsigned char SIGTERMs = 1;
unsigned char SIGINTs = 1;
unsigned char SIGSEGVs = 1;

int loops;

const char *cmd[] =
{
"ATE0",
"ATI",
"AT+GMR",
"AT+IPR=115200&W"
};

const char *msg_status[max_status] =
{
"Success",
"CRC16 error",
"Flash error",
"Download mode",
"Data package error",
"Unknown status"
};


unsigned short tbl[256] =
{
0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

const char *vio_up = "VIO_UP\n";
const char *vio_down = "VIO_DOWN\n";
char const * fillog = "wr_log.txt";
int fd, fd_timer;

unsigned short RK_index;
unsigned char RK_Mir, at_data = 0, Def_RK;

unsigned int tmr_ms, tmr_rx, tmr_tx, tmr0;
unsigned long long tmr_one;

s_COM COM;

unsigned char faza = 0, last_faza = 0;
unsigned char vio, last_vio;
unsigned char on_off;//1-on  0-off
unsigned char pwr_on_off = 0;//1-on  0-off

unsigned int seq_number = 0, seq_number_resp = 0xffffffff, pk_all;
int pk_now, pk_len, pk_status, file_size, last_size;
unsigned short pk_max_dl;

char txd[max_buf_len];
char rxd[max_buf_len];
char module[max_buf_len];
char module_new[max_buf_len];
char *uk_rxd = NULL;

unsigned char SYNC_WORD[2] = {0xB5, 0xA9};
unsigned char SYNC_WORD_RESP[2] = {0x5B, 0x9A};

unsigned char *bin = NULL;
int fsize = 0, fd_bin;
char fname[256];

char labka[128] = {0};

// ************************************************************************
// ************************************************************************
//			FUNCTION
// ************************************************************************
// ************************************************************************
unsigned int get_timer(unsigned int t)
{
unsigned char one[8];

    return (read(fd, one, 0) + t);

}
// **************************************************************************
int check_delay(unsigned int t)
{
unsigned char oned[8] = {0};//4

    if (read(fd, oned, 0) >= t ) return 1; else return 0;
}
//---------------------------- get date_and_time --------------------------
char *CurTime()
{
time_t ct;
char *abram;
    ct = time(NULL); abram = ctime(&ct); abram[strlen(abram) - 1] = 0;    return (abram);
}
//------------------------------------------------------------------------
char * DTNowPrn4(char *lab)
{
char st1[64] = {0};;
struct tm *ctimka;
time_t it_ct;
struct timeval tvl;
int msec, i_hour, i_min, i_sec;

    gettimeofday(&tvl, NULL);
    it_ct = tvl.tv_sec;
    ctimka = localtime(&it_ct);
    i_hour = ctimka->tm_hour; i_min = ctimka->tm_min; i_sec = ctimka->tm_sec; msec = (int)(tvl.tv_usec) / (int)1000;
    sprintf(st1,"%02d:%02d:%02d.%03d |", i_hour, i_min, i_sec, msec);
    msec = strlen(st1);
    memcpy(lab, st1, msec);

    return (lab);
}
//------------------------------------------------------------------------
void LogMsg(int LogLevel, const char * const Msg)
{
    if (LogLevel <= MaxLogLevel) syslog(LogLevel,"%s",Msg);
}
//------------------------------------------------------------------------
void PrintErrorSignal(int es)
{
char stz[128] = {0};

    sprintf(stz,"%sGOT SIGNAL : %d\n", DTNowPrn4(labka), es);
    LogMsg(LOG_INFO, stz);
    printf(stz);

}
// **************************************************************************
int print_msg(const char *st)
{
char stz[128] = {0};

    sprintf(stz,"%s ",DTNowPrn4(labka));
    printf(stz);
    printf(st);
    return 0;

}
// **************************************************************************
void string_AT_com(const char *st)
{
char stz[128] = {0};
const char *eolin = "\n";

    sprintf(stz,"%s ",DTNowPrn4(labka));
    printf(stz);
    printf(st);
    printf(eolin);

}
// **************************************************************************
//--------------------  function for processing SIGNAL from system -----------
void _SigProc(int sig)
{
    switch (sig) {
	case SIGHUP :
	    if (dmp) dmp = 0; else dmp = 1;
	break;
	case SIGTERM :
	    if (SIGTERMs) {
		SIGTERMs = 0;
		printf("\nSIGTERM : termination signal (term)\n");
		ErrorSignal = SIGTERM;
		PrintErrorSignal(ErrorSignal);
	    }
	    loops = 0;
	break;
	case SIGINT :
	    if (SIGINTs) {
		SIGINTs = 0;
		printf("\nSIGINT : interrupt from keyboard (int)\n");
		ErrorSignal = SIGINT;
		PrintErrorSignal(ErrorSignal);
	    }
	    loops = 0;
	break;
	case SIGSEGV :
	    if (SIGSEGVs) {
		SIGSEGVs = 0;
		printf("\nSIGSEGV : invalid memory reference (core)\n");
		ErrorSignal = SIGSEGV;
		PrintErrorSignal(ErrorSignal);
	    }
	    loops = 0;
	break;

    }
}
// **************************************************************************
void my_CloseAll()
{
    if (fd_bin > 0) close(fd_bin);
    if (bin) free(bin);
    if (fd_timer > 0) close(fd_timer);
    if (fd > 0) close(fd);
}
// ************************************************************************
//-----------------------------  create index adress for RK ---------------
void ind_rk(unsigned char nk)
{
unsigned short ix, i;
unsigned char ma[8];

    i = nk;
    RK_index = ((i >> 2) << 9) | ((i & 3) << 6);
    RK_Mir = 0xC8;//режим побайтного приема данных от РК + ModulePWR_OFF + 115200

    /* Reset subboard */
    ma[0] = 1;		//команда - запись одного байта
    ix = CS_RESET + ((i >> 2) << 9);
    ma[1] = ix;		// мл байт адреса смещения
    ma[2] = (ix >> 8);	// старший байт адреса смещения
    ma[3]=0xFE;		// data
    write(fd, ma, 4);	// RESET : write 0
    usleep(10000);
    ma[0] = 1;		//команда - запись одного байта
    ma[1] = ix;		// мл байт адреса смещения
    ma[2] = (ix >> 8);	// старший байт адреса смещения
    ma[3] = 0xFF; 	// data
    write(fd, ma, 4);	// RESET : write 1

}
// *************************************************************************
void SayToCore(unsigned char nk)
{
unsigned char cmd_present_rk[all_radio + 1] = {0};
int rtz;

    cmd_present_rk[0] = 0x0B;
    cmd_present_rk[nk + 1] = 1;

    rtz = write(fd, cmd_present_rk, all_radio + 1);

    if (rtz != all_radio) printf("Put to kernel level \"rk_present[]\" ERROR.\n");

}
// *************************************************************************
void print_faza(unsigned char sk)
{
char stra[128] = {0};

    sprintf(stra,"%s f=%03d\n", DTNowPrn4(labka), sk);
    printf(stra);

}
// *************************************************************************
void ModulePWR_ON(unsigned char nk)
{
const char *laka = "ModulePWR_ON\n";
char lll[128] = {0};

    if (!pwr_on_off) {

        RK_Mir &= (ModPwrOnOff ^ 0xFF);// bit0 = 0

        unsigned char ma[4] = {0x0E, nk, RK_Mir, 0};

        write(fd, ma, 3);

        pwr_on_off = 1;//1-on  0-off

        sprintf(lll, "%s [%02d] [%02X] %s", DTNowPrn4(labka), nk + 1, RK_Mir, laka);
        printf(lll);

    }

}
// *************************************************************************
//   процедура выключения питания модуля
// *************************************************************************
void ModulePWR_OFF(unsigned char nk)
{
const char *laka = "ModulePWR_OFF\n";
char lll[128] = {0};

    if (pwr_on_off) {

        RK_Mir = (RK_Mir | ModPwrOnOff); // bit0 = 1

        unsigned char ma[4] = {0x0E, nk, RK_Mir, 0};

        write(fd, ma, 3);

        pwr_on_off = 0;//1-on  0-off

        sprintf(lll, "%s [%02d] [%02X] %s", DTNowPrn4(labka), nk + 1, RK_Mir, laka);
        printf(lll);

    }

}
// *************************************************************************
//   процедура подачи пассивного уровня на вход ON/OFF модуля 
// *************************************************************************
void Module_ON(unsigned char nk)
{
const char *laka = "Module_ON\n";
char lll[128] = {0};

    RK_Mir |= ModOnOff;

    unsigned char ma[4] = {0x0E, nk, RK_Mir, 0};

    write(fd, ma, 3);

    on_off = 1;//1-on  0-off

    sprintf(lll, "%s [%02d] [%02X] %s", DTNowPrn4(labka), nk + 1, RK_Mir, laka);
    printf(lll);

}
// *************************************************************************
//   процедура подачи активного уровня на вход ON/OFF модуля GR47/PiML
// *************************************************************************

void Module_OFF(unsigned char nk)
{
const char *laka = "Module_OFF\n";
char lll[128] = {0};

    RK_Mir &= (ModOnOff ^ 0xFF); // bit0 = 1

    unsigned char ma[4] = {0x0E, nk, RK_Mir, 0};

    write(fd, ma, 3);

    on_off = 0;//1-on  0-off

    sprintf(lll, "%s [%02d] [%02X] %s", DTNowPrn4(labka), nk + 1, RK_Mir, laka);
    printf(lll);

}
// *************************************************************************
unsigned char ChangeSpeed(unsigned char nk)
{
const char *laka = "ChangeSpeed\n";
unsigned char bt = RK_Mir & 0xC0;
char lll[128] = {0};

    if (!bt) RK_Mir |= 0xC0;//set 115200
        else RK_Mir &= 0x3F;//set 9600

    unsigned char ma[4] = {0x0E, nk, RK_Mir, 0};

    write(fd, ma, 3);

    sprintf(lll, "%s [%02d] [%02X] %s", DTNowPrn4(labka), nk + 1, RK_Mir, laka);
    printf(lll);

    return (RK_Mir & 0xC0);//=0 - 9600,  !=0 - 115200

}
// **************************************************************************
void reset_timer_tmr(unsigned char tp)//tmr_mod <- clear long_long
{
unsigned char m[8] = {0};

    switch (tp) {
        case 0: m[0] = 4; break;//сбросить 1 ms таймер (unsigned long long)
        case 1: m[0] = 8; break;//сбросить 10-ти ms таймер (unsigned long)
            default : m[0] = 2; //сбросить оба таймера : 10ms (unsigned long) + 1ms (unsigned long long)
    }
    if (fd_timer > 0) write(fd_timer, m, 1);

    return;
}
//*******************************************************************************
int check_timer_tmr(unsigned long long t)//tmr_mod
{
unsigned char m[8];
unsigned int rc;
unsigned long long tm;

    rc = read(fd_timer, m, 8);
    if (rc != 8) return 0;
    else {
        memcpy(&tm, &m[0], 8);
        if (tm >= t) return 1; else return 0;
    }

}
// **************************************************************************
unsigned long long get_timer_tmr(unsigned long long t)//tmr_mod
{
unsigned char m[8];
unsigned int rc;
unsigned long long tm;

    rc = read(fd_timer, m, 8);
    if (rc != 8) return 0;
    else {
        memcpy(&tm, &m[0], 8);
        return (tm + t);
    }

}
// **************************************************************************
unsigned char myhextobin(char st, char ml)
{
char line[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
unsigned char a, b, c, i;

    for (i = 0; i < 16; i++) { if (st == line[i]) { b = i; break; } else b = 255; }

    for (i = 0; i < 16; i++) { if (ml == line[i]) { c = i; break; } else c = 255; }

    if ((b == 255) || (c == 255)) a = 255; else { b = b << 4;   a = b | c; }

    return a;
}
// *************************************************************************
unsigned char check_vio(unsigned char nk)
{
unsigned int res = nk;
unsigned char bu[4] = {0};
char lll[128];

    res = (res << 8) | 9;
    read(fd, bu, res);
    vio = (bu[0] & VIO);

    if (vio != last_vio) {
	sprintf(lll, "%s [%02d] [%02X] ", DTNowPrn4(labka), nk + 1, RK_Mir);
	if (vio) sprintf(lll+strlen(lll),"%s", vio_up);
	    else sprintf(lll+strlen(lll),"%s", vio_down);
	printf(lll);
	last_vio = vio;
    }
    return vio;

}
// *************************************************************************
void put_AT_com(const char *st)
{

    memset(COM.TX_com, 0, max_buf_len); COM.tx_numb=0;
    sprintf((char *)COM.TX_com, "\r\n%s\r\n", st);
    COM.tx_numb = strlen((char *)COM.TX_com);
    COM.tx_faza = 1;

}
// *************************************************************************
void put_AT_data(unsigned char *dat, unsigned short len)
{
unsigned short dl = len;

    if (dl > 1024) dl = 1024;
    memcpy(COM.TX_com, dat, dl);
    COM.tx_numb = dl;
    COM.tx_faza = 1;

}
// *************************************************************************
int RXdone(unsigned char nk)
{
unsigned int rz = nk;
unsigned char bulka[8] = {0}, bt;
int len, ret = 0;

    rz <<= 8; rz |= 9;
    len = read(fd, bulka, rz);
    if (len > 0) {
	bt = bulka[0] & RDEMP;
	if (bt != 0) ret = 0;//нет данных для чтения
	        else ret = 1;//есть данные
    }

    return ret;

}
//----------------------------------------------------------------------
unsigned char RXbyte_prn(unsigned char nk, unsigned char prn_bt, unsigned char scr)
{
unsigned int res = nk;
unsigned char bu[8] = {0};
int ret = 0;
unsigned char rt = 0;

    res <<= 8; res |= 0x13;//new function

    ret = read(fd, bu, res);

    if (ret == 1) {
	rt = bu[0];
	if (scr) {
	    if (rt == prn_bt) printf("%s [%02d] RX : %02X\n", DTNowPrn4(labka), nk + 1, rt);
	}
    }

    return rt;
}
//----------------------------------------------------------------------
int RXbyte(unsigned char nk, unsigned char scr)
{
unsigned int res = nk;
unsigned char bu[8] = {0};
int ret = 0, rt = -1;

    res <<= 8; res |= 0x13;//new function

    ret = read(fd, bu, res);

    if (ret == 1) {
	rt = bu[0];
	if (scr) printf("%s [%02d] RX : %02X\n", DTNowPrn4(labka), nk + 1, rt);
    }

    return rt;
}
// *************************************************************************
int RXbytes22(unsigned char nk, char * outs, unsigned char at)//at=0 - at_command, else - data
{
char bu[1026] = {0};
unsigned int len, rz = 0, i;

#ifdef x86
    if (!at) i = 0x19;//ANSWER READ, конец пакета 0x0d  +   16-ти разрядное чтение (готовность + данных)
	else i = 0x89;//DATA READ
#else
    if (!at) i = 0x19;//ANSWER READ, конец пакета 0x0d  +   16-ти разрядное чтение (готовность + данных)
	else i = 0x99;//DATA READ
#endif

    rz = (((rz | nk) << 8) | i);

    len = read(fd, bu, rz);

    if (len > 0) {
	len &= 1023;
	memcpy(outs, bu, len);
    } else len = 0;

    return len;

}
// *************************************************************************
int TXdone(unsigned char nk)
{
unsigned int res = nk;
unsigned char bu[8] = {0};

    res <<= 8; res |= 9;

    if (read(fd, bu, res) > 0) {
	if ((bu[0] & WREMP) != 0) return 1; else return 0;
    } else return 0;

}
// *************************************************************************
int TXdone2(unsigned char nk)
{
unsigned int res = nk;
unsigned char bu[4] = {0};

    res <<= 8; res |= 0x0F;

    return (read(fd, bu, res));

}
// *************************************************************************
int TXbyte(unsigned char nk, unsigned char bb, unsigned char fl)
{
unsigned char ob[4] = {0x1a, nk, bb, 0};
char stra[128];
int ret = 0;

    ret = write(fd, ob, 3);

    if (fl) {
	if (ret == 1) sprintf(stra,"%s [%02d] TX : %02X\n", DTNowPrn4(labka), nk + 1, bb);
	         else sprintf(stra,"%s [%02d] TX : %02X ERROR (ret=%d)\n", DTNowPrn4(labka), nk + 1, bb, ret);
	printf(stra);
    }

    return ret;
}
// ************************************************************************
int TXbyte_all(unsigned char nk, unsigned char *st, unsigned short len)
{
unsigned char ob[1026] = {0};//514
int i;

    i = len + 2; if (i > 1026) i = 1026;//514;
    ob[0] = 0x0f;//0x19;//0x16
    ob[1] = nk;
    memcpy(&ob[2], st, i - 2);

    return (write(fd, ob, i));

}
// ************************************************************************
unsigned char TXCOM2_all(unsigned char nk, unsigned char at)//at=0 - at_command, else - data
{
unsigned char tx_sk = 2;
char *stx = NULL, *uk;
unsigned short dl;
int rt = 0;

    if (TXdone2(nk)) {
	dl = COM.tx_numb;
        rt = TXbyte_all(nk, &COM.TX_com[0], dl);
        if ((rt > 0) && (rt <= 1024)) COM.tx_numb = rt;
	if (!at) {
	    if ((COM.TX_com[0] == 0x0a) || (COM.TX_com[0] == 0x0d)) {
		uk = (char *)&COM.TX_com[2];
		dl = rt - 2;
	    } else {
		uk = (char *)&COM.TX_com[0];
		dl = rt - 1;
	    }
	    COM.TX_com[dl] = 0;
	    stx = calloc(1, max_buf_len + 16);
	    if (stx) {
		sprintf(stx,"tx: %s", uk);
		string_AT_com(stx);
		free(stx);
	    }
	}
	tx_sk = 0;
    }

    return tx_sk;
}
// ************************************************************************
void SetTickMode(unsigned char tmode)
{
unsigned char ob[4];

#ifdef x86
    ob[0] = 0x10;// Set/Unset DSS_ONLY
#else
    ob[0] = 0xFF;// timeout restart MyTimer
#endif
    ob[1] = tmode;
    write(fd, ob, 2);

}
// ************************************************************************
unsigned short mk_crc16(unsigned char *data, unsigned short len)
{
unsigned short word = 0, i;

    for (i = 0; i < len; i++ ) word = (word << 8) ^ (unsigned short)tbl[((word >> 8) ^ *data++) & 0xff];

    return word;
}
// ************************************************************************
int mk_dl_begin(unsigned char prn)
{
/*typedef struct{
unsigned char head;
unsigned short cmd; \
unsigned short len;  > CRC16
unsigned int data;  /
unsigned short crc16;
} s_dl_begin;*/
s_dl_begin tmp;
int ret = sizeof(s_dl_begin), i;
char *stx = NULL;
unsigned char *uk;

    tmp.head  = MARKER;					//1	AA
    tmp.cmd   = htons(CMD_DL_BEGIN);			//2	00 01
    tmp.len   = htons((unsigned short)sizeof(int));	//2	00 04
    tmp.data  = htonl(VERSION);				//4	00 00 00 01
    tmp.crc16 = mk_crc16((unsigned char *)&tmp.cmd, 8);	//2	XX XX
    tmp.crc16 = htons(tmp.crc16);

    stx = calloc(1, 512);
    if (stx) {
	uk = (unsigned char *)&tmp.head;
	sprintf(stx+strlen(stx),"%s BEGIN (%d): ", DTNowPrn4(labka), ret);
	if (prn) for (i = 0; i < ret; i++) sprintf(stx+strlen(stx),"%02X ",*(unsigned char *)(uk++));
	sprintf(stx+strlen(stx),"\n\thead\t%02X\n\tcmd\t%04X\n\tlen\t%04X\n\tdata\t%08X\n\tcrc16\t%04X",
					tmp.head, ntohs(tmp.cmd), ntohs(tmp.len), ntohl(tmp.data), ntohs(tmp.crc16));
	sprintf(stx+strlen(stx),"\n");
	printf(stx);
	free(stx);
    }

#ifndef TEST_MODE
    put_AT_data((unsigned char *)&tmp.head, (unsigned short)ret);
#endif

    return ret;

}
// ************************************************************************
int mk_dl_end(unsigned char prn)
{
/*typedef struct{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short crc16;
} s_dl_end;*/
s_dl_end tmp;
int ret = sizeof(s_dl_end), i;
char *stx = NULL;
unsigned char *uk;

    tmp.head  = MARKER;					//1	AA
    tmp.cmd   = htons(CMD_DL_END);			//2	00 05
    tmp.len   = 0;//htons((unsigned short)sizeof(int));	//2	00 00
    tmp.crc16 = mk_crc16((unsigned char *)&tmp.cmd, 4);	//2	XX XX
    tmp.crc16 = htons(tmp.crc16);

    stx = calloc(1, 512);
    if (stx) {
	uk = (unsigned char *)&tmp.head;
	sprintf(stx+strlen(stx),"%s END (%d): ", DTNowPrn4(labka), ret);
	if (prn) for (i = 0; i < ret; i++) sprintf(stx+strlen(stx),"%02X ",*(unsigned char *)(uk++));
	sprintf(stx+strlen(stx),"\n\thead\t%02X\n\tcmd\t%04X\n\tlen\t%04X\n\tcrc16\t%04X",
					tmp.head, ntohs(tmp.cmd), ntohs(tmp.len), ntohs(tmp.crc16));
	sprintf(stx+strlen(stx),"\n");
	printf(stx);
	free(stx);
    }

#ifndef TEST_MODE
    put_AT_data((unsigned char *)&tmp.head, (unsigned short)ret);
#endif

    return ret;

}
// ************************************************************************
int mk_run_gsmsw(unsigned char prn)
{
/*typedef struct{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short crc16;
} s_run_gsmsw;*/
s_dl_end tmp;
int ret = sizeof(s_run_gsmsw), i;
char *stx = NULL;
unsigned char *uk;

    tmp.head  = MARKER;					//1	AA
    tmp.cmd   = htons(CMD_RUN_GSMSW);			//2	00 07
    tmp.len   = 0;//htons((unsigned short)sizeof(int));	//2	00 00
    tmp.crc16 = mk_crc16((unsigned char *)&tmp.cmd, 4);	//2	XX XX
    tmp.crc16 = htons(tmp.crc16);

    stx = calloc(1, 512);
    if (stx) {
	uk = (unsigned char *)&tmp.head;
	sprintf(stx+strlen(stx),"%s RUN_GSMSW (%d): ",DTNowPrn4(labka),ret);
	if (prn) for (i = 0; i < ret; i++) sprintf(stx+strlen(stx),"%02X ",*(unsigned char *)(uk++));
	sprintf(stx+strlen(stx),"\n\thead\t%02X\n\tcmd\t%04X\n\tlen\t%04X\n\tcrc16\t%04X",
					tmp.head, ntohs(tmp.cmd), ntohs(tmp.len), ntohs(tmp.crc16));
	sprintf(stx+strlen(stx),"\n");
	printf(stx);
	free(stx);
    }

#ifndef TEST_MODE
    put_AT_data((unsigned char *)&tmp.head, (unsigned short)ret);
#endif

    return ret;

}
// ************************************************************************
int mk_dl_data(unsigned char prn, int len, unsigned int s_n)
{
/*typedef struct{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned int seq_num;
//unsigned char *data;
//unsigned short crc16;
} s_dl_data;*/
s_dl_data tmp;
int ret, i, j, dl = sizeof(s_dl_data);
char *stx = NULL;
unsigned short crc16;
unsigned char *out = NULL, *uk = NULL, *dta = NULL;
unsigned int sqn;

    tmp.head  = MARKER;					//1	AA
    tmp.cmd   = htons(CMD_DL_DATA);			//2	00 03				- cmd
    tmp.len   = htons(len + 4);				//2	?? ??				- len
//    tmp.seq_num = htonl(s_n);				//4	?? ?? ?? ?? (from 0....)	- seq_number
							//(len)	.. .. .. .. .. .. .. .. .. ..	- data
							//2	XX XX  				- crc16
    ret = dl + len + 2;
    out = calloc(1, ret);
    if (out && bin) {
	j = ret; if (j > 24) j = 24;
	dta = bin + (s_n * (max_data_len + 4));
	memcpy((unsigned char *)&sqn, dta, 4);
	tmp.seq_num = htonl(sqn);
	memcpy(out, (unsigned char *)&tmp.head, dl);
	memcpy(out + dl, dta + 4, len);
	crc16 = mk_crc16((unsigned char *)(out + 1), dl + len - 1);
	crc16 = htons(crc16);
	memcpy(out + dl + len, (unsigned char *)&crc16, 2);
	stx = calloc(1, 1024);
	if (stx) {
	    uk = out;
	    sprintf(stx+strlen(stx),"%s DATA (%d): ", DTNowPrn4(labka), ret);
	    if (prn) {
		for (i = 0; i < j; i++) sprintf(stx+strlen(stx),"%02X ",*(unsigned char *)(uk++));
		sprintf(stx+strlen(stx),"\n\thead\t%02X\n\tcmd\t%04X\n\tlen\t%04X\n\tseq_num\t%08X\n\tdata\t%d bytes\n\tcrc16\t%04X",
					tmp.head, ntohs(tmp.cmd), ntohs(tmp.len), ntohl(tmp.seq_num), len, ntohs(crc16));
	    } else sprintf(stx+strlen(stx),"\tcmd=%d, len=%04d, seq_num=%06d, data=%d bytes",
					ntohs(tmp.cmd), ntohs(tmp.len), ntohl(tmp.seq_num), len);
	    sprintf(stx+strlen(stx),"\n");
	    printf(stx);
	    free(stx);
	}

#ifndef TEST_MODE
	put_AT_data(out, (unsigned short)ret);
#endif

	free(out);

    } else ret = 0;

    return ret;

}
// ************************************************************************
int parse_answer(unsigned char prn, unsigned char *data, int len, unsigned int *s_n, unsigned short *m_d)
{
/*typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short status;
} s_resp;
typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short status;
unsigned short crc16;
} s_ack;
#define CMD_DL_BEGIN_RESP	0x0002
#define CMD_DL_DATA_RESP	0x0004
#define CMD_DL_END_RESP		0x0006
#define CMD_RUN_GSMSW_RESP	0x0008
*/
//s_resp *tmp=NULL;
int data_len, i, dl, status = -1;
char *stx = NULL;
unsigned char *uk, bt = 0, yes = 0;
unsigned short calc, max_dl;
unsigned int sn;


    uk = data; bt = *uk;//must be 0xAA    !!!!!!!!!!!!!!
    if ((len < sizeof(s_resp) + 2) || (bt != 0xAA)) {
	printf("%s ERROR_RESP (%d): ", DTNowPrn4(labka), len);
	for (i = 0; i < len; i++) printf("%02X ",*(unsigned char *)(uk++));
	printf("\n");
    } else {
	memset((unsigned char *)&hdr_resp.head,0,sizeof(s_ack));
	memcpy((unsigned char *)&hdr_resp.head, data, sizeof(s_ack) - 2);
	hdr_resp.cmd = ntohs(hdr_resp.cmd);
	hdr_resp.len = ntohs(hdr_resp.len);
	hdr_resp.status = ntohs(hdr_resp.status);
	data_len = hdr_resp.len - 2;
	memcpy((unsigned char *)&hdr_resp.crc16, data+len - 2, 2);
	hdr_resp.crc16 = ntohs(hdr_resp.crc16);
	calc = mk_crc16((unsigned char *)(data + 1), len - 3);
	status = hdr_resp.status;
	dl = (len << 2) + 256;
	stx = calloc(1, dl);
	if (stx) {
	    switch (hdr_resp.cmd) {
		case CMD_DL_BEGIN_RESP  : sprintf(stx,"%s BEGIN_RESP (%d): ",DTNowPrn4(labka), len);  yes = 1; break;
		case CMD_DL_DATA_RESP   : sprintf(stx,"%s DATA_RESP (%d): ",DTNowPrn4(labka), len);   if (prn) yes = 1; break;
		case CMD_DL_END_RESP    : sprintf(stx,"%s END_RESP (%d): ",DTNowPrn4(labka), len);    yes = 1; break;
		case CMD_RUN_GSMSW_RESP : sprintf(stx,"%s RUN_GSMSW_RESP (%d): ",DTNowPrn4(labka), len); yes = 1; break;
		    default : { sprintf(stx,"%s UNKNOWN_RESP (%d): ",DTNowPrn4(labka), len); yes = 1; }
	    }
	    if (prn) {
		for (i = 0; i < len; i++) sprintf(stx+strlen(stx),"%02X ",*(unsigned char *)(uk++));
		sprintf(stx+strlen(stx),"\n");
		if (yes) sprintf(stx+strlen(stx),"\thead\t%02X\n\tcmd\t%04X\n\tlen\t%04X\n\tstatus\t%04X",
							hdr_resp.head, hdr_resp.cmd, hdr_resp.len, status);
	    } else {
		if (yes) sprintf(stx+strlen(stx),"\n\thead\t%02X\n\tcmd\t%04X\n\tlen\t%04X\n\tstatus\t%04X",
						hdr_resp.head, hdr_resp.cmd, hdr_resp.len, status);
		    else sprintf(stx+strlen(stx),"\tcmd=%d, len=%04d",hdr_resp.cmd, hdr_resp.len);
	    }
	    if (data_len > 0) {
		if ((hdr_resp.cmd == CMD_DL_DATA_RESP) && (data_len == 4)) {
		    memcpy((unsigned char *)&sn, data+sizeof(s_resp), 4);
		    sn = ntohl(sn);
		    *s_n = sn;
		    if (prn) sprintf(stx+strlen(stx),"\n\tseq_num\t%08X",sn);
			else sprintf(stx+strlen(stx),", seq_num=%06d, status=%d",sn,status);
		} else if ((hdr_resp.cmd == CMD_DL_BEGIN_RESP) && (data_len == 2)) {
		    memcpy((unsigned char *)&max_dl, data+sizeof(s_resp), 2);
		    max_dl = ntohs(max_dl);
		    *m_d = max_dl;
		    if (yes) sprintf(stx+strlen(stx),"\n\tmax_len\t%04X (%d)", max_dl, max_dl);
		} else if (yes) sprintf(stx+strlen(stx),"\n\tdata\t%d bytes", data_len);
	    }
	    if (yes) sprintf(stx+strlen(stx),"\n\tcrc16\t%04X\n", hdr_resp.crc16);
	    if (calc != hdr_resp.crc16) sprintf(stx+strlen(stx),"CRC16 ERROR: recv=%04X calc=%04X", hdr_resp.crc16, calc);
	    if ( *(stx+strlen(stx) - 1) != '\n') sprintf(stx+strlen(stx),"\n");
	    printf(stx);
	    free(stx);
	} else {
	    printf("%s Memory calloc(%dl) ERROR\n", DTNowPrn4(labka), dl);
	    status = -1;
	}
    }

    return status;

}
// ************************************************************************
int ReadBIN(char *fn)
{
int ret = -1, dl = -1, rt;
struct stat sb;
unsigned char *uk = NULL;

    fd_bin = open(fname, O_RDONLY, 0664);
    if (fd_bin > 0) {
	if (!fstat(fd_bin, &sb)) {
	    file_size = dl = sb.st_size;
	    if (dl > 0) {
		pk_all = dl / max_data_len;
		last_size = dl % max_data_len;
		if (last_size > 0) pk_all++;
		dl += (pk_all << 2);//  + seq_num for each packet
		pk_now = 0;
		bin = calloc(1, dl);
		if (bin) {
		    uk = bin;
		    while (pk_now < pk_all) {
			memcpy(uk, (unsigned char *)&pk_now, 4); uk += 4;
			rt = read(fd_bin, uk, max_data_len);
			if (rt == max_data_len) {
			    uk += rt;
			    pk_now++;
			} else {
			    if (rt > 0) pk_now++;
			    break;
			}
		    }
		    ret = dl;
		}
	    }
	}
	close(fd_bin);
    }

    return ret;
}
// ************************************************************************
void ClearRX()
{
int cnt = 0;

    printf("%s Start clear rx_buf ...", DTNowPrn4(labka));
    while (RXdone(Def_RK)) { RXbyte(Def_RK, 0); cnt++; if (cnt >= max_buf_len) break; }
    printf(" done.(%d)\n",cnt);

}
// ************************************************************************
int mk_ack_data(unsigned int s_n, unsigned char *out)
{
/*typedef struct
{
unsigned char head;
unsigned short cmd;
unsigned short len;
unsigned short status;
unsigned int seq_num;
unsigned short crc16;
} s_ack_data;
unsigned char ack_data[13] = {0xaa,0x00,0x04,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0xf5,0x82,0x70};
*/
s_ack_data ack;
unsigned short lens = 6;
int dl = sizeof(s_ack_data);

    ack.head  = MARKER;					//1	AA
    ack.cmd   = htons(CMD_DL_DATA_RESP);		//2	00 04				- cmd
    ack.len   = htons(lens);				//2	00 06				- lens
    ack.status = 0;					//2	00 00				- status
    ack.seq_num = htonl(s_n);				//4	?? ?? ?? ?? (from 0....)	- seq_number
    ack.crc16 = htons( mk_crc16((unsigned char *)&ack.cmd, dl - 3) );
    memcpy(out, (unsigned char *)&ack.head, dl);

    return dl;

}
// ************************************************************************
void SetKT(unsigned char nk)
{
unsigned char ob[4] = {0x0d, nk, 0, 0};
int rt = 0;

    rt = write(fd, ob, 2);
    printf("%s SetKT: %d set for %d subboard\n", DTNowPrn4(labka), nk + 1, rt);

}
// ************************************************************************

