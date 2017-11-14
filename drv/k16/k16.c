//-----------------------------------------------------------------------------------------------------------
//  Linux kernel device driver for Altera chipset and Infenion Vinetic DSP on board polygator (g20 gateway)
//-----------------------------------------------------------------------------------------------------------

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/atomic.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/time.h>

#undef KERNEL_OLD	//2.6.26
#define KERNEL_NEW	//3.1.6

#define VIN_REG_STATUS
//#undef VIN_REG_STATUS

#undef DSP_NOT_READY

//#define PRN_LINE
#undef PRN_LINE

#define x86

//#define WAIT_AT_TX_READY
#undef WAIT_AT_TX_READY

//-----------------------------------------------------------------
#define max_brd		4
#define max_card	max_brd * 2

#define all_radio	max_card * 4
//#define MyHZ		100	// для таймера 10 милисекунд
#define Max_Len_Rom	128
#define SIZE_BUF	4096	//32768//65536
#define RDEMP		2       //  бит статуса "буфер для чтения COM-порта готов"
#define WREMP		4       //  бит статуса "буфер для записи в COM-порт готов"
#define RDEMPPROG	0x40    //  бит статуса "буфер для чтения COM-порта2 готов"
#define MaxLenWrite 	40960
#define MOD_RDempty  	8      	//  бит статуса "буфер для чтения COM-порта SIM готов"
#define MOD_WRempty  	0x10   	//  бит статуса "буфер для записи в COM-порт SIM готов"
#define msk_id		0x0ff0	// A903,B902,C905,D904,E907,F906  - CARD ID

#define AdrTest16	0x11B0	//16-ти разрядный регистр на субплате (запись-чтение 16-ти разрядов)
#define AdrTest16_2	0x1180	//16-ти разрядный регистр на субплате (запись-чтение 16-ти разрядов)

#define CS_PRESENCE   	0x11E0 	//  адрес идентификатора плат  
#define CS_ROM   	0x11D0 //  адрес идентификатора прошивки на субплате
#define CS_RESET   	0x11F0 //  базовый адрес сброса плат  
#define CROSS_BANK_MODE 0x2300 //  базовый адрес регистра управления режимом : с банком - 0, без банка - 1
#define G8_ONLY		0x6000//  = 0 - крылья (24 канала),   = 1 - только G4 или G8
#define KT_POINT	0x1140
#define CS_PiML_AT_COM 	0x1010 //  базовый адрес управления COM-портами AT (данные)  
#define CS_PiML_SIM_COM 0x1020 //  базовый адрес управления COM-портами модулей (1020, 1060, 10A0, 10E0)
#define CS_PiML_STATUS 	0x1000 //  базовый адрес управления/чтения статуса модулей 
//read : 7 6 5 4 3 2 1 0	write : 7 6 5 4 3 2 1 0
//	 W R R W R W R V		\-/ \-/ M S O P
//       R D S R D R D I                 S   S  O P N W
//           T         O                 P   P  D E / R
//       I I   S S E E                   E   E  E E O
//       M M S I I M M                   E   E    D F
//       E E I M M P P                   D   D    2 F
//       I I M                           1   3
//					SPEED1:00-9600, 10-28800, 11-115200 (для CS_PiML_AT_COM)
//					SPEED3:00-9600, 01-50k,   10-100k (для CS_PiML_SIM_COM)
//				                MODE:0-обычный режим приемо-предатчика АТ команд модуля
//						     1-программирование(смена прошивки) модуля
//
//						  SPEED2: не используется
//
//						    ON/OFF вкл./выкл. модуля
//
//						      PWR - вкл./выкл. питания модуля
//
//#define CS_MIRROR 	0x1032 //  базовый адрес управления адресами буферов Алтеры (sim & at)
////	write : 7 6 5 4 3 2 1 0
////		        w r w r
////		        r d r d
////		        a a s s
////			t t i i
////			    m m
#define CS_MIRROR_ATR 	0x1034 //
//	write : 7 6 5 4 3 2 1 0
//			      atr reset
#define CS_MIRROR_R_SIM	0x1028//0x1036 //
//	write : 7 6 5 4 3 2 1 0
//			      rd sim
#define CS_MIRROR_W_SIM	0x1038 //
//	write : 7 6 5 4 3 2 1 0
//			      wr sim
#define CS_MIRROR_R_AT	0x102C//0x103A //
//	write : 7 6 5 4 3 2 1 0
//			      rd at
#define CS_MIRROR_W_AT	0x103C //
//	write : 7 6 5 4 3 2 1 0
//			      wr at
//**********************   FOR VINETIC   ***********************************************
#define kol_card	max_card

#define CS_VINETIC   		0x1104 //  базовый адрес обращения к VINETIC
#define CS_VINETICe   		0x1108 //  базовый адрес обращения к VINETIC
#define CS_RESET_VINETIC   	0x11C0 //  адрес сброса VINETIC-а на субплате
#define CS_VIN_ADR   		0x1170

#ifdef VIN_REG_STATUS
    #define CS_STATUS_REGISTR	0x14	//0x1104 + 0x14 = 0x1118
    #define CS_STATUS_REGISTRe	0x28	//0x1108 + 0x28 = 0x1130
    #define MASK_PIN   		0x80
#else
    #define CS_STATUS_REGISTR	0x1C	//0x1104 + 0x1C = 0x1120
    #define MASK_PIN   		1
#endif


#define Ready_Count	30
#define len_buff 2100
#define cntr_def 5000//5 sec

#define max_rtp_pack 32
#define rd_rings max_rtp_pack/8
#define wr_max_rtp_pack 32
#define wr_rings wr_max_rtp_pack/8
#define max_len_record 256

//*************************************************************************************
#ifdef KERNEL_NEW
    #define CLASS_DEV_CREATE(class, devt, device, name) device_create(class, device, devt, NULL, "%s", name)
    #define CLASS_DEV_DESTROY(class, devt) device_destroy(class, devt)
#else
    #define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, _name)
    #define CLASS_DEV_DESTROY(_class, _devt) device_destroy(_class, _devt)
#endif
static struct class *altera_class = NULL;
static struct class *vin_class = NULL;

/* macro-oni for determining a unit (channel) number */
#define	UNIT(file) MINOR(file->f_dentry->d_inode->i_rdev)
#define	NODE(file) MAJOR(file->f_dentry->d_inode->i_rdev)

#define DevName "k16"
#define DevNameV "vin"

#define buf_size 2048
#define blk_size 2048
#define timi_def 5
#define timi_def2 10

#define kols	21

#define bar_count 2

//--------------------------------------------------------------------------------------
typedef struct 
{
    void __iomem *BaseAdrRAM;
    unsigned char num;
    unsigned char adr;
    unsigned char adr10;
    unsigned short id;
    unsigned char type;//0-pci, 1-pcie
} s_subcard;


static void __iomem *BaseAdr[max_brd] = {NULL};
static unsigned char len_block_read=32;
static unsigned char *ibuff=NULL;
static unsigned char *ibuffw=NULL;

static unsigned short RK_index[all_radio];
static unsigned int RK_len_buff[all_radio];

static unsigned int my_msec;
static struct timer_list my_timer; 
static int my_timer_ready=0;
static int my_dev_ready=0;

static atomic_t varta;

static int Major=0; 
module_param(Major, int, 0);
static int MajorV=0; 
module_param(MajorV, int, 0);

static int Device_Open[kol_card+1] = {0};

struct rchan_sio {
  struct cdev cdev;
  struct cdev cdevV;
};

static unsigned char sim900_buf[all_radio][buf_size];
static atomic_t sim900_len[all_radio];
static atomic_t sim900_flag[all_radio];
static atomic_t sim900_type[all_radio];
static atomic_t sim900_txd[all_radio];

static unsigned char rk_present[all_radio]={0};

static unsigned char tcard=0;
static unsigned char subboard[max_card]={0};
static s_subcard subcard[max_card];

static int timi=timi_def;

static atomic_t tick_mode;

//-----------  For PCI  -------------------------------------
static struct pci_dev *gsm_board[max_brd];
static unsigned int bar_start[max_brd][bar_count];
static unsigned int bar_len[max_brd][bar_count];
static unsigned int bar_flag[max_brd][bar_count];

//**********************   FOR VINETIC   ***********************************************
static unsigned int VinAdrData[kol_card];
static unsigned int VinAdrStat[kol_card];
static unsigned int VinAdrAdr[kol_card];

static int cntr[kol_card] = {cntr_def};

static spinlock_t vin_lock[kol_card];
static spinlock_t sre_lock[kol_card];

static atomic_t ops[kol_card];
static atomic_t ips[kol_card];

static atomic_t bxsr[kol_card];
static atomic_t hwsr[kol_card];

static int b_rtp=0;

static int subboard_flag;

static atomic_t begin_rtp[kol_card];

static wait_queue_head_t poll_wqh[kol_card];

static unsigned char sre_buff[kol_card][32];

//static unsigned char next=2;//2 //restart timer - 2msec.

static int zero_def=500;//1 sec

static int zero[kol_card];

//-----------------------

typedef struct
{
unsigned short len;
unsigned short body[max_len_record];
} s_one_rtp;

typedef struct
{
int head;
int tail;
int ring;
s_one_rtp rtp[max_rtp_pack];
} s_all_packs;

typedef struct
{
int head;
int tail;
int ring;
s_one_rtp rtp[wr_max_rtp_pack];
} s_wr_all_packs;


typedef struct 
{
unsigned short type_chan;//номер канала винетика в младшем байте
unsigned short len;//длинна пакета в словах с заголовком в 6 слов(12 байт)
} s_hdr_rtp;
s_hdr_rtp *hdr_rtp;

#define len_vrm 512

//-----------------------------------------------------
static spinlock_t rd_lock[kol_card];
static s_all_packs all_packs[all_radio];
static unsigned char vrm_rd[kol_card][len_vrm];
static atomic_t ops_list[all_radio];
static atomic_t rtp_on[all_radio];
static atomic_t rtp_init[all_radio];
static atomic_t rtp_init2[all_radio];

static spinlock_t wr_lock[kol_card];
static s_wr_all_packs wr_all_packs[kol_card];
static unsigned char vrm_wr[kol_card][len_vrm];
static atomic_t ips_list[kol_card];
//-----------------------------------------------------

static atomic_t dss_only;

static atomic_t rd_error[kol_card];
static atomic_t wr_error[kol_card];

static atomic_t evt_rtp[kol_card][4];
static atomic_t param_rtp[kol_card][4];

static atomic_t rk_rtp[kol_card][4];
static atomic_t rk2_rtp[kol_card][4];
static atomic_t done_rtp[kol_card][4];
static atomic_t set_mix_param_rtp[kol_card][4];
static atomic_t set_mix2_param_rtp[kol_card][4];

static atomic_t near_echo_param[kol_card][4];
static atomic_t far_echo_param[kol_card][4];
static atomic_t set_near_echo_param[kol_card][4];
static atomic_t set_far_echo_param[kol_card][4];

static atomic_t gsm_flag[all_radio];

static atomic_t to_vin[all_radio];
static atomic_t from_vin[all_radio];

static unsigned int ssrc=0;

//************************************************************
//************************************************************
void PrnSubCard(unsigned char np)
{
char stx[256];

    if (np<max_card) {
	if (subboard[np]) {
	    memset(stx,0,256);
	    sprintf(stx,"\t[%d]: Adr=%x num=%d adr/adr10=%x/%x id=%x type=%d",
		np+1,
		(unsigned int)subcard[np].BaseAdrRAM,
		subcard[np].num+1,
		subcard[np].adr,
		subcard[np].adr10,
		subcard[np].id,
		subcard[np].type);
	    if (subcard[np].type)  sprintf(stx+strlen(stx)," (PCIe)");
			  else {
			    if (subcard[np].BaseAdrRAM) sprintf(stx+strlen(stx)," (PCI)");
						   else sprintf(stx+strlen(stx)," (none)");
			  }
	    sprintf(stx+strlen(stx)," VinAdr=%x/%x/%x\n",
			(unsigned int)VinAdrData[np],
			(unsigned int)VinAdrStat[np],
			(unsigned int)VinAdrAdr[np]);
	    printk(KERN_ALERT "%s",stx);
	}
    }
}
//************************************************************
unsigned char AdrToNum(unsigned int ix)
{
unsigned int ofs;
unsigned char ad, np, num=255;

    ofs = ix;
    ofs >>= 8;
    ofs &= 0xff;
    ad = ofs;
    for (np=0; np<max_card; np++) {
	if (ad == subcard[np].adr) {
	    num = subcard[np].num;
	    break;
	}
    }

    return num;
}
//************************************************************
void __iomem *BasedByAdr(unsigned int ix)
{
void __iomem *adr=NULL;
unsigned int ofs;
unsigned char ad, np;

    ofs = ix;
    ofs >>= 8;
    ofs &= 0xff;
    ad = ofs;
    for (np=0; np<max_card; np++) {
	if ((ad == subcard[np].adr) || (ad == subcard[np].adr10)) {
	    adr = subcard[np].BaseAdrRAM;
	    break;
	}
    }

    return adr;
}
//************************************************************
void __iomem *BasedByCard(unsigned char np)
{
void __iomem *adr=NULL;
unsigned char ad, i;

    if (np>=max_card) return adr;

    ad = np;
    for (i=0; i<max_card; i++) {
	if (ad == subcard[i].num) {
	    adr = subcard[i].BaseAdrRAM;
	    break;
	}
    }

    return adr;
}
//************************************************************
void __iomem *BasedByRK(unsigned char rk)
{
void __iomem *adr=NULL;
unsigned char ad, i;

    if (rk>=all_radio) return adr;

    ad = rk>>2;
    for (i=0; i<max_card; i++) {
	if (ad == subcard[i].num) {
	    adr = subcard[i].BaseAdrRAM;
	    break;
	}
    }

    return adr;
}
//************************************************************
void reset_subcard(unsigned char np)
{
unsigned int ix;

    if (!subboard[np]) return;

    ix = subcard[np].adr;
    ix <<= 8;
    ix |= (CS_RESET & 0x00ff);
    if (subcard[np].type) ix <<= 4;

    iowrite8(0,subcard[np].BaseAdrRAM+ix);
	mdelay(10);
    iowrite8(1,subcard[np].BaseAdrRAM+ix);
}
//*******************************************************************************
void reset_vinetic(unsigned char np)
{
unsigned int ix;

    if (!subboard[np]) return;

    ix = subcard[np].adr;
    ix <<= 8;
    ix |= (CS_RESET_VINETIC & 0x00ff);
    if (subcard[np].type) ix<<=4;

    iowrite8(0,subcard[np].BaseAdrRAM+ix);
	mdelay(10);
    iowrite8(1,subcard[np].BaseAdrRAM+ix);
}
//************************************************************
int tx_buf(unsigned char nk)
{
int total, i, j;
unsigned int ix,ixa,ixb;
unsigned char bt;
void __iomem *adra=NULL;

    i = 0;
    total = atomic_read(&sim900_len[nk]);
    i     = atomic_read(&sim900_txd[nk]);
    j = total - i; if (j>blk_size) j=blk_size;
    ix  = RK_index[nk] + CS_PiML_AT_COM;
    ixa = RK_index[nk] + CS_PiML_STATUS;
    ixb = RK_index[nk] + CS_MIRROR_W_AT;
//    adra = BasedByAdr(ix);
    adra = BasedByRK(nk);
    if (subcard[nk>>2].type) {
	ix  <<=4;
	ixa <<=4;
	ixb <<=4;
    }
    if (adra!=NULL) {
	while (j>0)  {
#ifdef WAIT_AT_TX_READY
	    if ((ioread8(adra+ixa)) & WREMP) {
#endif
		iowrite8(sim900_buf[nk][i],adra+ix);
		j--; i++;
		bt=0; iowrite8(bt,adra+ixb);
		bt=1; iowrite8(bt,adra+ixb);
#ifdef WAIT_AT_TX_READY
	    } else break;
#endif
	}
    }
    atomic_set(&sim900_txd[nk], i);

    return i;
}
//************************************************************
void read_rom_subcard(unsigned char np, char *uk, unsigned char flg)
{
unsigned char bbb, bt;
int i,j,lp;
unsigned int ix;
char stx[256];

    if (!subboard[np]) return;

    ix = subcard[np].adr;
    ix <<= 8;
    ix |= (CS_ROM & 0x00ff);
    if (subcard[np].type) ix<<=4;

    bbb=0;
    iowrite8(bbb,subcard[np].BaseAdrRAM+ix); 
    bt = ioread8(subcard[np].BaseAdrRAM+ix); 
    bbb++;
    j=Max_Len_Rom; i=0; lp=1; memset(stx,0,256);
    while (lp) {
	iowrite8(bbb,subcard[np].BaseAdrRAM+ix); 
	bt = ioread8(subcard[np].BaseAdrRAM+ix);
	stx[i] = bt;
	i++;  j--;
	if ((!bt) || (!j)) lp=0;
	bbb++;
    }
    i=strlen(stx);
    if ((uk!=NULL) && (i>0))  memcpy(uk,stx,i);

    if (flg) printk(KERN_ALERT "%s: subboard %d : ID=%04X ROM=(0x%X)='%s'\n",
				DevName,np+1,subcard[np].id,ix,stx);

}
//************************************************************
#ifdef PRN_LINE
static char my_timer_str[32] = {0};
static char *uk_str=NULL;
//*******************************************************************************
static char *my_time_now(void)
{
struct timeval tm;

    do_gettimeofday(&tm);

    memset(my_timer_str, 0,32);
    sprintf(my_timer_str,"[%08d.%03d]",(unsigned int)tm.tv_sec, (unsigned int)tm.tv_usec/1000);

    uk_str = &my_timer_str[0];

    return uk_str;
}
#endif
//*******************************************************************************
static void clr_all_packs(unsigned char nk)
{
    atomic_set(&ops_list[nk], 0);

    all_packs[nk].head=0;
    all_packs[nk].tail=0;
    all_packs[nk].ring=0;
}
// ************************************************************************
static int put_to_rtp_list(unsigned char nk, unsigned char *uk, int len)
{
int tail, ret;

    tail = all_packs[nk].tail;
    memcpy(&all_packs[nk].rtp[tail].body[0], uk, len);
    all_packs[nk].rtp[tail].len=len;//это в байтах
    all_packs[nk].ring++;
    tail++;		if (tail==max_rtp_pack) tail=0;		all_packs[nk].tail=tail;
    atomic_set(&ops_list[nk], 1);
    ret=all_packs[nk].ring;
    return ret;
}
// ************************************************************************
static int get_from_rtp_list(unsigned char nk, unsigned char *uk)
{
int head, tail, dl;

    head = all_packs[nk].head;	tail = all_packs[nk].tail;
    if (all_packs[nk].ring==0)  {
	atomic_set(&ops_list[nk], 0);
	return 0;
    }
    dl = all_packs[nk].rtp[head].len;
    memcpy(uk, &all_packs[nk].rtp[head].body[0], dl);
    if (all_packs[nk].ring > 0) all_packs[nk].ring--;
    head++; if (head==max_rtp_pack) head=0;
    all_packs[nk].head = head;
    return dl;
}
//*******************************************************************************
static void wr_clr_all_packs(unsigned char np)
{
    atomic_set(&ips_list[np], 0);

    wr_all_packs[np].head=0;
    wr_all_packs[np].tail=0;
    wr_all_packs[np].ring=0;

}
// ************************************************************************
static int wr_put_to_rtp_list(unsigned char np, unsigned char *uk, int len)
{
int tail, ret=0;

    tail = wr_all_packs[np].tail;
    memcpy(&wr_all_packs[np].rtp[tail].body[0], uk, len);
    wr_all_packs[np].rtp[tail].len=len;//это в байтах
    wr_all_packs[np].ring++; 
    tail++;		if (tail==wr_max_rtp_pack) tail=0;	wr_all_packs[np].tail=tail;

    return ret;
}
// ************************************************************************
static int wr_get_from_rtp_list(unsigned char np, unsigned char *uk)
{
int head, dl=0, tail;

    head = wr_all_packs[np].head;	tail = wr_all_packs[np].tail;
    if (head==tail) {
	atomic_set(&ips_list[np], 0);
	wr_all_packs[np].ring=0;
	return 0;
    }
    dl = wr_all_packs[np].rtp[head].len;
    memcpy(uk, &wr_all_packs[np].rtp[head].body[0], dl);

    return dl;
}
// ************************************************************************
static int wr_check_rtp_write(unsigned char np, int to, int from)
{
int head, errs=0;

    if (to==from) {
	
	head = wr_all_packs[np].head;

	head++; if (head==wr_max_rtp_pack) head=0;

	wr_all_packs[np].head = head;

	if (wr_all_packs[np].ring > 0) wr_all_packs[np].ring--; else wr_all_packs[np].ring = 0;
	
    } else errs=1;

    return errs;

}
// ************************************************************************
//
// ************************************************************************
static int wr_vineticT(unsigned short dta, unsigned char npl, unsigned char idxa)
{
unsigned char sho;
unsigned short b_s, ind=0;
int loop=1, ok=0, ret=0;
#ifdef DSP_NOT_READY
unsigned int err;
#endif
void __iomem *adra=NULL;
void __iomem *adrad=NULL;
void __iomem *adradr=NULL;
#ifdef VIN_REG_STATUS
unsigned short adi;
#endif

    sho=Ready_Count;	ind=idxa;
    adra = (void __iomem *)VinAdrStat[npl];
    adrad = (void __iomem *)VinAdrData[npl];
    adradr = (void __iomem *)VinAdrAdr[npl];
    while (loop) {
#ifdef VIN_REG_STATUS
	if (subcard[npl].type) {
	    adi = (unsigned short)VinAdrStat[npl];
	    iowrite16(adi, adradr);
	    udelay(1);
	    b_s = ioread16(adrad);
	} else b_s = ioread16(adra);
#else
	b_s = ioread16(adra);
#endif
	if ((b_s & MASK_PIN)==0) { loop=0; ok=1; break; } else sho--;
	if (sho==0) loop=0;
    }

    if (ind==2) ind=4;
    if (subcard[npl].type) ind <<= 4;

    iowrite16(dta,adrad+ind);		//udelay(1);

    if (ok) ret=0;
    else {
#ifdef DSP_NOT_READY
	err=b_s;	atomic_set(&wr_error[npl], err|0xE0000000);
#endif	
#ifdef PRN_LINE
	printk(KERN_ALERT "[vin%d] wr_vineticT: dsp not ready[a_stat=%x a_data=%x a_adr=%x ind=%x]\n",
		    npl,
		    VinAdrStat[npl],
		    VinAdrData[npl],
		    VinAdrAdr[npl],
		    ind);
#endif
	ret=1;

    }

    return ret;
}
//----------------------------------------------------------------
static unsigned short rd_vineticT(unsigned char npl, unsigned char idxa)
{
unsigned short rt=0, ind=0;
unsigned char sho;
unsigned short b_s;
int loop, ok=0;
#ifdef DSP_NOT_READY
unsigned int err;
#endif
void __iomem *adra=NULL;
void __iomem *adrad=NULL;
void __iomem *adradr=NULL;
#ifdef VIN_REG_STATUS
unsigned short adi;
#endif

    sho=Ready_Count;    loop=1;	ok=0;	ind=idxa;
    adra = (void __iomem *)VinAdrStat[npl];
    adrad = (void __iomem *)VinAdrData[npl];
    adradr = (void __iomem *)VinAdrAdr[npl];
    while (loop) {
#ifdef VIN_REG_STATUS
	if (subcard[npl].type) {
	    adi = (unsigned short)VinAdrStat[npl];
	    iowrite16(adi, adradr);
	    udelay(1);
	    b_s = ioread16(adrad);
	} else b_s = ioread16(adra);
#else
	b_s = ioread16(adra);
#endif
	if ((b_s & MASK_PIN)==0) { loop=0; ok=1; break; } else sho--;
	if (sho==0) loop=0;
    }

    if (ind==2) ind=4;
#ifdef VIN_REG_STATUS
    if (subcard[npl].type) {
	ind <<= 4;
	iowrite16((unsigned short)(VinAdrData[npl]+ind), adradr);
	udelay(1);
	rt=ioread16(adrad);
    } else rt=ioread16(adrad+ind);
#else
    rt=ioread16(adrad+ind);
#endif

//    udelay(1);

    if (!ok) {
#ifdef DSP_NOT_READY
	err=b_s;	atomic_set(&wr_error[npl], err|0xF0000000);
#endif	
#ifdef PRN_LINE
    #ifdef VIN_REG_STATUS
	printk(KERN_ALERT "[vin%d] rd_vineticT: dsp not ready[a_stat=%x a_data=%x a_adr=%x ind=%x adi=%x b_s=%x]\n",
		    npl,VinAdrStat[npl],VinAdrData[npl],VinAdrAdr[npl],ind,adi,b_s);
    #else
	printk(KERN_ALERT "[vin%d] rd_vineticT: dsp not ready[a_stat=%x a_data=%x a_adr=%x ind=%x b_s=%x]\n",
		    npl,VinAdrStat[npl],VinAdrData[npl],VinAdrAdr[npl],ind,b_s);
    #endif
#endif
    }// else
//    printk(KERN_ALERT "[vin%d] rd_vineticT: a_stat=%x a_data=%x a_adr=%x ind=%x adi=%x b_s=%x\n",
//		    npl,VinAdrStat[npl],VinAdrData[npl],VinAdrAdr[npl],ind,adi,b_s);

    return rt;
}
//************************************************************
static int check_cerr(unsigned char npl)
{
int ret=0;
unsigned short bxsr1, bxsr2;
unsigned int mores;


spin_lock_bh(&vin_lock[npl]);
    udelay(4);
    wr_vineticT(0xC040, npl, 0);
    bxsr1 = (rd_vineticT(npl,0) & 0x100);
    bxsr2 = (rd_vineticT(npl,2) & 0x26);
    if (bxsr1 & 0x100) {
	wr_vineticT(0x0600, npl, 0);
	wr_vineticT(0xA000, npl, 2);
	ret=1;
    }
    if (bxsr2 & 0x26) {
	wr_vineticT(0x4330, npl, 2);
	ret=1;
    }
spin_unlock_bh(&vin_lock[npl]);

    mores=bxsr1;	mores=(mores<<16) | bxsr2;
    atomic_set(&bxsr[npl],mores);

    return ret;
}
//************************************************************
static int check_mbox(unsigned char npl, unsigned char kol)
{
unsigned short stat;
int ret=0, dta=0;

spin_lock_bh(&vin_lock[npl]);
    udelay(2);
    wr_vineticT(0xC100, npl, 0);
    stat = rd_vineticT(npl, 2);
spin_unlock_bh(&vin_lock[npl]);

    stat>>=8;
    if (stat<kol) {
	ret=1;
	dta=0x80000000|stat;
#ifdef PRN_LINE
	printk(KERN_ALERT "%s[vin%d] check_mbox: no space: need=%d, got=%d\n", my_time_now(), npl, kol, stat);
#endif
    }

    atomic_set(&wr_error[npl], dta);

    return ret;
}
//************************************************************
static int start_all_rtp(unsigned char npl, unsigned char ch, unsigned char alm_pcm)
{
int ret=0;
unsigned short st, ml, rk;//, nk;

    if (!atomic_read(&begin_rtp[npl])) return ret;

    ssrc++;	ml=ssrc;    st=ssrc>>16;
    rk = (npl << 2) + ch;
//    if (alm_pcm) nk = ch;
//	    else nk = rk&7;

    if (!check_mbox(npl, 9)) {

spin_lock_bh(&vin_lock[npl]);
	wr_vineticT(0x0600, npl, 0);	// COM: Coder Chan. Configuration (RTP) page 292
	wr_vineticT(0x7002, npl, 0);
	wr_vineticT(0x0000, npl, 0);	//TIMESTAMP HW
	wr_vineticT(0x0000, npl, 2);	//TIMESTAMP LW
udelay(10);//10
	wr_vineticT(0x0600+ch, npl, 0);	// COM: Coder Chan. Configuration (RTP) page 296
	wr_vineticT(0x7103, npl, 0);
	wr_vineticT(st, npl, 0);	//SSRC
	wr_vineticT(ml, npl, 0);	//SSRC
	wr_vineticT(0x0000, npl, 2);	//SEQ-NR
spin_unlock_bh(&vin_lock[npl]);

	//udelay(4);//4
	check_cerr(npl);
	    atomic_set(&rk_rtp[npl][ch], 0);
	    atomic_set(&rk2_rtp[npl][ch], 0);
	    ret=1;
	    atomic_set(&rtp_on[rk], 1);

    }
#ifdef PRN_LINE
	else printk(KERN_ALERT "%s[vin%d|%d|%d] start_all_rtp fail\n", my_time_now(), npl, ch, rk);
#endif

    return ret;//0 - не выполнено,    1 - выполнено

}
//************************************************************
static int init_all_mix_param_rtp(unsigned char npl, unsigned char ch, unsigned char alm_pcm)
{
int ret=0;
unsigned short sho, slovo, word, wrd, rk, word2, st, ml, tslot;//, wrd2;
unsigned int param, param2;

    if (!atomic_read(&begin_rtp[npl])) return ret;

    rk = (npl << 2) + ch;
    slovo = ch&3;
    tslot = rk&7;

    param = atomic_read(&param_rtp[npl][ch]);//VAD+CN and GAIN
    sho = param>>16;//VAD+CN
    word=param;//GAIN
    if (alm_pcm) wrd = slovo+0x11;//ALM
	    else wrd = tslot+1;//PCM
    wrd += (0x9000 + (slovo<<8));

    param2 = atomic_read(&evt_rtp[npl][ch]);
    word2=param2;
    ssrc++;	ml=ssrc;    st=ssrc>>16;

    if (!check_mbox(npl, 11)) {

	spin_lock_bh(&vin_lock[npl]);
	    wr_vineticT(0x0600+ch, npl, 0);//COM: ?Coder Channel Speech Compression? [M-V]? on Page 282 
	    wr_vineticT(0x6103, npl, 0);//0x6103
	    wr_vineticT(wrd, npl, 0); //Enable IP
	    //здесь необходимо установить кодек и payload time, полученные от sipua //	VAD + Noise
	    wr_vineticT(sho, npl, 0);	//         GAIN   SET
	    wr_vineticT(word, npl, 2);//page 236  GAIN : RK / PCM
	    udelay(16);
	    wr_vineticT(0x0600+ch, npl, 0);
	    wr_vineticT(0x5004, npl, 0);
	    wr_vineticT(st, npl, 0);//SSRC
	    wr_vineticT(ml, npl, 0);//SSRC
	    wr_vineticT(0x0000, npl, 0);//SEQ-NR
	    wr_vineticT(word2, npl, 2);//EVT-PT
	spin_unlock_bh(&vin_lock[npl]);

	//udelay(4);
	check_cerr(npl);
	    atomic_set(&set_mix_param_rtp[npl][ch], 0);
	    atomic_set(&set_mix2_param_rtp[npl][ch], 0);
	    ret=1;
	    atomic_set(&rtp_init[rk], 0);
	    atomic_set(&rtp_init2[rk], 1);
	
    } 
#ifdef PRN_LINE
    else printk(KERN_ALERT "%s[vin%d|%d|%d] init_all_mix_param_rtp fail\n", my_time_now(), npl, ch, rk);
#endif

//printk(KERN_ALERT "COMMUT: vin=%d chan=%d rk=%d ts=%d\n", npl, ch, rk, tslot);

    return ret;//0 - не выполнено,    1 - выполнено

}
//************************************************************
static int do_near_echo(unsigned char npl, unsigned char ch)
{
int ret=0;
unsigned short sho, word, rk;
unsigned param;

    if (!atomic_read(&begin_rtp[npl])) return ret;

    rk = (npl << 2) + ch;

    if (!check_mbox(npl, 3)) {

	param = atomic_read(&near_echo_param[npl][ch]);
	sho = param>>16;	word=param;

	spin_lock_bh(&vin_lock[npl]);
	    wr_vineticT(0x0600+ch, npl, 0);
	    wr_vineticT(sho, npl, 0);
	    wr_vineticT(word, npl, 2);
	spin_unlock_bh(&vin_lock[npl]);

	//udelay(4);
	check_cerr(npl);
	ret=1;
	atomic_set(&set_near_echo_param[npl][ch], 0);
    }
#ifdef PRN_LINE
    else printk(KERN_ALERT "%s[vin%d|%d|%d] do_near_echo fail\n", my_time_now(), npl, ch, rk);
#endif

    return ret;//0 - не выполнено,    1 - выполнено
}
//************************************************************
static int do_far_echo(unsigned char npl, unsigned char ch)
{
int ret=0;
unsigned short sho, word, rk;
unsigned param;

    if (!atomic_read(&begin_rtp[npl])) return ret;

    rk = (npl << 2) + ch;

    if (!check_mbox(npl, 3)) {

	param = atomic_read(&far_echo_param[npl][ch]);
	sho = param>>16;	word=param;

	spin_lock_bh(&vin_lock[npl]);
	    wr_vineticT(0x0600+ch, npl, 0);
	    wr_vineticT(sho, npl, 0);
	    wr_vineticT(word, npl, 2);
	spin_unlock_bh(&vin_lock[npl]);

	//udelay(4);
	check_cerr(npl);
	atomic_set(&set_far_echo_param[npl][ch], 0);
	ret=1;
    }
#ifdef PRN_LINE
    else printk(KERN_ALERT "%s[vin%d|%d|%d] do_near_echo fail\n", my_time_now(), npl, ch, rk);
#endif


    return ret;//0 - не выполнено,    1 - выполнено
}
//************************************************************
static int do_done_rtp(unsigned char npl, unsigned char ch)
{
int ret=0;
unsigned short sho, rk;

    if (!atomic_read(&begin_rtp[npl])) return ret;

    rk = (npl << 2) + ch;

    if (!check_mbox(npl, 4)) {

	sho = ch;

	spin_lock_bh(&vin_lock[npl]);
	    wr_vineticT(0x0600+ch, npl, 0);//COM: ?Coder Channel Speech Compression? [M-V]? on Page 282 
	    wr_vineticT(0x6102, npl, 0);
	    wr_vineticT(0x1000+(sho<<8), npl, 0);//Disable IP c коммутацией на null
	    wr_vineticT(0, npl, 2); //Disable DEC
	spin_unlock_bh(&vin_lock[npl]);

	//udelay(4);
	check_cerr(npl);
	ret=1;

    } 
#ifdef PRN_LINE
    else printk(KERN_ALERT "%s[vin%d|%d|%d] do_done_rtp: close rtp fail\n", my_time_now(), npl, ch, rk);
#endif

    atomic_set(&done_rtp[npl][ch], 0);
    atomic_set(&set_near_echo_param[npl][ch], 0);
    atomic_set(&set_far_echo_param[npl][ch], 0);
    atomic_set(&rk_rtp[npl][ch], 0);
    atomic_set(&rk2_rtp[npl][ch], 0);
    atomic_set(&rtp_on[rk], 0);
    atomic_set(&rtp_init[rk], 0);
    atomic_set(&rtp_init2[rk], 0);
    spin_lock_bh(&rd_lock[npl]);
	clr_all_packs(rk);
    spin_unlock_bh(&rd_lock[npl]);

    return ret;//0 - не выполнено,    1 - выполнено

}
//************************************************************
static int do_425(unsigned char npl, unsigned char ch, unsigned char f)
{
int ret=0;
unsigned short sho;
unsigned char nk,rk;

    if (!atomic_read(&begin_rtp[npl])) return ret;

    switch (f) {
	case 1:
	    sho=0x0000;//TG1 OFF
	break;
	case 2:
	    sho=0x001D;//TG1 ON
	break;
	    default : return ret;
    }

    nk = ch;
    rk = (npl << 2) + nk;

    if (!check_mbox(npl, 3)) {
	spin_lock_bh(&vin_lock[npl]);
	    wr_vineticT(0x0100+nk, npl, 0);
	    wr_vineticT(0x0901, npl, 0);
	    wr_vineticT(sho, npl, 2);//TG1 OFF/ON
	spin_unlock_bh(&vin_lock[npl]);
	//udelay(4);//2
	check_cerr(npl);
	ret=1;
    }
#ifdef PRN_LINE
    else printk(KERN_ALERT "%s[vin%d|%d|%d] do_425: fail\n", my_time_now(), npl, ch, rk);
#endif

    return ret;//0 - не выполнено,    1 - выполнено

}
//************************************************************
static int check_dsp(unsigned char npl)
{
int ret=0;
unsigned short bxsr1, bxsr2, hwsr1, hwsr2;
unsigned int mores, mores2;

    spin_lock_bh(&vin_lock[npl]);
	udelay(4);
	wr_vineticT(0xC040, npl, 0);
	bxsr1 = (rd_vineticT(npl,0) & 0x100);
	bxsr2 = (rd_vineticT(npl,2) & 0x26);
	if (bxsr1 & 0x100) {	// COM: Command CERR Ask. (page 379). 
	    wr_vineticT(0x0600, npl, 0);
	    wr_vineticT(0xA000, npl, 2);
	}
	if (bxsr2 & 0x26)		// COM: Confirmation ERROR (wPHIERR) page57 
	    wr_vineticT(0x4330, npl, 2);
	if (bxsr2 & 1) ret=1;	//d0=1 in-box empty - можно писать; d0=0 - писать нельзя

	wr_vineticT(0xC020, npl, 0);
	hwsr1 = (rd_vineticT(npl,0) & 0x0b89);
	hwsr2 = (rd_vineticT(npl,2) & 1); 
	if (hwsr1) wr_vineticT(0x4320, npl, 2);
    spin_unlock_bh(&vin_lock[npl]);

    mores=bxsr1;	mores=(mores<<16) | bxsr2;
    mores2=hwsr1;	mores2=(mores2<<16) | hwsr2;

    atomic_set(&bxsr[npl],mores);	atomic_set(&hwsr[npl],mores2);

    return ret;
}
//************************************************************
static int check_dspT(unsigned char npl)
{
int ret=0;
unsigned short bxsr1, bxsr2, hwsr1, hwsr2;
unsigned int mores, mores2;


    spin_lock(&vin_lock[npl]);
	udelay(4);
	wr_vineticT(0xC040, npl, 0);
	bxsr1 = (rd_vineticT(npl,0) & 0x100);
	bxsr2 = (rd_vineticT(npl,2) & 0x26);
	if (bxsr1 & 0x100) {	// COM: Command CERR Ask. (page 379). 
	    wr_vineticT(0x0600, npl, 0);
	    wr_vineticT(0xA000, npl, 2);
	}
	if (bxsr2 & 0x26)	// COM: Confirmation ERROR (wPHIERR) page57 
	    wr_vineticT(0x4330, npl, 2);
	if (bxsr2 & 1) ret=1;	//d0=1 in-box empty - можно писать; d0=0 - писать нельзя

	wr_vineticT(0xC020, npl, 0);
	hwsr1 = (rd_vineticT(npl,0) & 0x0b89);
	hwsr2 = (rd_vineticT(npl,2) & 1);
	if (hwsr1) wr_vineticT(0x4320, npl, 2);
    spin_unlock(&vin_lock[npl]);

    mores=bxsr1;	mores=(mores<<16) | bxsr2;
    mores2=hwsr1;	mores2=(mores2<<16) | hwsr2;

    atomic_set(&bxsr[npl],mores);	atomic_set(&hwsr[npl],mores2);

    return ret;
}
//************************************************************
static int rd_pack(unsigned char vi, unsigned char vi_ops)
{
unsigned char ops_len, vi2;
unsigned short ofs, wrd, sho;
unsigned int err=0;
int ret=0, mores, i, valid, rk2;
unsigned char * uk;

    if (vi_ops<=8) err = 0x80000000;

    check_dspT(vi);

    ofs = vi;    ops_len=vi_ops;	mores=1;

    spin_lock(&vin_lock[vi]);
	memset(&vrm_rd[vi][0],0,len_vrm);
	wr_vineticT(0xC120, vi, 0);
	while (mores) {
	    wrd = rd_vineticT(vi, 0);
	    uk=&vrm_rd[vi][ret];	*(unsigned short *)uk = wrd;
	    ret+=2;
	    ops_len--;	if (ops_len==1) mores=0;
	}
	wrd = rd_vineticT(vi, 2);
	uk=&vrm_rd[vi][ret];	*(unsigned short *)uk = wrd;	ret+=2;
    spin_unlock(&vin_lock[vi]);

    if ((ret>0) && (ret<=512)) {
	i = 0; valid=1; wrd=0;
	while (valid) {
	    hdr_rtp = (s_hdr_rtp *)&vrm_rd[vi][i];
	    wrd = (vi<<2) + (hdr_rtp->type_chan&3);//номер радиоканала 0..31//23
	    sho = hdr_rtp->len & 0x00ff;
	    if (sho==255) {
		err=0x2fff0000|sho;
		break;
	    }
	    ofs = sho;
	    sho += 2;
	    sho <<=1;//полный размер пакета в байтах
	    if ((sho>16) && (sho<=512)) {//176 - максимальный размер пакета для g711 в байтах
		if (atomic_read(&rtp_on[wrd])) {
		    atomic_inc(&from_vin[wrd]);
		    rk2 = atomic_read(&gsm_flag[wrd]); 
		    if (rk2>all_radio) {
			spin_lock(&rd_lock[vi]);
			    mores = put_to_rtp_list(wrd, &vrm_rd[vi][i], sho);
			spin_unlock(&rd_lock[vi]);
		    } else {
			vi2 = rk2>>2;
			hdr_rtp->type_chan &= 0x7ffc;
			hdr_rtp->type_chan |= (unsigned short)(rk2&3); 
			spin_lock(&wr_lock[vi2]);
			    wr_put_to_rtp_list(vi2, &vrm_rd[vi][i], sho);
			    atomic_set(&ips_list[vi2], 1);
			spin_unlock(&wr_lock[vi2]);
			atomic_inc(&to_vin[rk2]);
		    }
		} else err=0x40000000|wrd;
		i+=sho;
		if (i>=ret) valid=0;
	    } else { 
		valid=0; err|=0x20000000; err|=ofs; 
	    }
	}
    } else err|=0x10000000;

    atomic_set(&rd_error[vi], err);

    return (ret>>1);
}
//************************************************************
static int wr_pack(unsigned char vi)
{
int mores, mores2, ret=0, i, br, j;
unsigned char l_ips, c=0;
unsigned short wrd, slovo;//, word;
unsigned int err=0;
unsigned char * uk;

    uk=&vrm_wr[vi][0];
    spin_lock(&wr_lock[vi]);
	mores2 = wr_get_from_rtp_list(vi, uk);
    spin_unlock(&wr_lock[vi]);

    if (mores2==0) goto bye;

    check_dspT(vi);

    wrd=mores2>>1;
    br=0;  mores=1; i=60;
    spin_lock(&vin_lock[vi]);
	while (mores) {
	    c 		= wr_vineticT(0xC100, vi, 0);
	    l_ips 	= rd_vineticT(vi,2);
	    if (wrd <= l_ips) { mores=0; br=1; } else udelay(1);
	    i--;	if (!i) mores=0;
	}
    spin_unlock(&vin_lock[vi]);

    if (!br) {
	err=(wrd<<8)|l_ips;
	if (c) err|=0x80000000;
	goto bye;
    }

    j=wrd;//количество слов для записи в винетик
    word=vi;	i=0;	mores=1;
    spin_lock(&vin_lock[vi]);
	while (mores) {
	    memcpy(&slovo, uk+i, 2);
	    wr_vineticT(slovo, vi, 0); 
	    i+=2;	j--;	if (j==1) mores=0;
	}
	memcpy(&slovo, uk+i, 2);
	wr_vineticT(slovo, vi, 2);
    spin_unlock(&vin_lock[vi]);
    ret=i+2;

    spin_lock(&wr_lock[vi]);
	mores = wr_check_rtp_write(vi, mores2, ret);
    spin_unlock(&wr_lock[vi]);
	if (mores) err=0x40000000|mores;

bye:

    atomic_set(&wr_error[vi], err);

    return (ret>>1);
}
//************************************************************
static int read_sre(unsigned char npl)//read SRE[16]
{
int ret=0, i;
unsigned char v_buff[32];
unsigned short wrd;

    spin_lock_bh(&vin_lock[npl]);
	wr_vineticT(0xC010, npl, 0);
	for (i=0; i<15; i++) {
	    wrd=rd_vineticT(npl,0);
	    memcpy(&v_buff[ret], &wrd, 2);
	    ret+=2;
	}
	wrd=rd_vineticT(npl,2);
    spin_unlock_bh(&vin_lock[npl]);
	memcpy(&v_buff[ret], &wrd, 2);
	ret+=2;
	if (ret>32) ret=32;

    spin_lock_bh(&sre_lock[npl]);
	memcpy(&sre_buff[npl][0],v_buff,ret);
    spin_unlock_bh(&sre_lock[npl]);

    return ret;
}
//************************************************************
//************************************************************
//************************************************************
//		    таймер 2 mcek
//************************************************************
void MyTimer(unsigned long d)
{
int rk, dl, fl, total;
unsigned char k, l_ops=0;
int l=0, l0, m, t_m, delta;
unsigned char mas[kol_card];


    t_m = atomic_read(&tick_mode); delta = 10 / t_m;
    timi--;
    if (!timi) { timi=t_m; atomic_inc(&varta); }

/**/
    if (!atomic_read(&dss_only)) {
	memset(mas,0,kol_card);
	for (k=0; k<kol_card; k++) {
	    if ((subboard[k]) && (atomic_read(&begin_rtp[k]))) {

		spin_lock(&vin_lock[k]);
		    wr_vineticT(0xC110, k, 0);
		    l_ops = rd_vineticT(k,2);
		spin_unlock(&vin_lock[k]);
		if (l_ops) rd_pack(k, l_ops);

		m=0;
		spin_lock(&wr_lock[k]);
		    l = atomic_read(&ips_list[k]);
		    m = wr_all_packs[k].ring;
		spin_unlock(&wr_lock[k]);
		if (m <= wr_rings) mas[k]=1;
		if (l) wr_pack(k);

		if (!mas[k]) {
		    m=0;
		    spin_lock(&rd_lock[k]);
			for (l0=0; l0<4; l0++) m+=atomic_read(&ops_list[(k<<2)+l0]);
		    spin_unlock(&rd_lock[k]);
		    mas[k] = m;
		}
	    }
	}
	for (k=0; k<kol_card; k++) {
	    if ((subboard[k]) && (atomic_read(&begin_rtp[k]))) {
		if (mas[k]) wake_up_interruptible(&poll_wqh[k]);
	    }
	}
    }
/**/
/**/
    for (rk=0; rk<all_radio; rk++) {
	if (rk_present[rk]) {
	    if (atomic_read(&sim900_type[rk])) {
		dl = atomic_read(&sim900_len[rk]);
		fl = atomic_read(&sim900_flag[rk]);
		if ((dl>0) && (!fl)) {
		    total = tx_buf(rk);
		    if (total>=dl) atomic_set(&sim900_flag[rk], 1);//буфер пуст, в него можно помещать новые данные
		} else atomic_set(&sim900_flag[rk], 1);//буфер пуст, в него можно помещать новые данные
	    }
	}
    }
/**/

    my_timer.expires = jiffies + delta;//next;	//2 mсек.
    mod_timer(&my_timer, jiffies + delta);

    return;
}
//***********************************************************
//                открытие устройства
//************************************************************
static int rchan_open(struct inode *inode, struct file *filp)
{
struct rchan_sio *sio;

int unit, un, ret=-ENODEV, b=0, rk, node;

    node = NODE(filp);
    unit = UNIT(filp);

    if (node==Major) {
	un=unit;
	if (Device_Open[0]) return -EBUSY;	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	Device_Open[0]++;			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	sio = container_of(inode->i_cdev, struct rchan_sio, cdev);
	filp->private_data = sio;
	return 0;
    }
    else
    if (node==MajorV) {
	un=unit+1;
	if ((unit >= 0) && (unit < kol_card)) {
	    if (!Device_Open[un]) {
		Device_Open[un]++;
		sio = container_of(inode->i_cdev, struct rchan_sio, cdevV);
		filp->private_data = sio;
		atomic_set(&begin_rtp[unit], (int)0);
		init_waitqueue_head(&poll_wqh[unit]);
		for (rk=0; rk<4; rk++) {
		    clr_all_packs((unit<<2)+rk);
		    atomic_set(&ops_list[unit], b);
		}
		wr_clr_all_packs(unit);
		atomic_set(&ips_list[unit], b);
		return 0;
	    } else ret = -EBUSY;
	}
	return ret;
    }

    return ret;
}
//***********************************************************
//                   закрытие устройства
//************************************************************
static int rchan_release(struct inode *inode, struct file *filp) 
{
int unit, un, b=0, rk, node;

    node = NODE(filp); un = unit = UNIT(filp);

    if (node==Major) {
	if (Device_Open[0]>0) Device_Open[0]--;
	return 0;
    }
    else
    if (node==MajorV) {
	un = unit+1;
	if (Device_Open[un]>0) Device_Open[un]--;
	if ((unit >= 0) && (unit < kol_card)) {
	    atomic_set(&begin_rtp[unit], 0);
	    for (rk=0; rk<4; rk++) {
		clr_all_packs((unit<<2)+rk);
		atomic_set(&ops_list[unit], b);
	    }
	    wr_clr_all_packs(unit);
	    atomic_set(&ips_list[unit], b);
	    return 0;
	} else return -1;
    }

    return 0;
}
//************************************************************
static unsigned int rchan_poll(struct file *filp, struct poll_table_struct *wait_table)
{
int ret, unit, l, m, rk;

    if (NODE(filp)!=MajorV) return 0;

    unit = UNIT(filp);
    ret=0;

    if ((unit >= 0) && (unit < kol_card)) {

	poll_wait(filp, &poll_wqh[unit], wait_table);

	l = m = 0;

	if ( (subboard[unit]) && (atomic_read(&begin_rtp[unit])) ) {

	    spin_lock_bh(&rd_lock[unit]);
		for (rk=0; rk<4; rk++) l+=atomic_read(&ops_list[(unit<<2)+rk]);
	    spin_unlock_bh(&rd_lock[unit]);

	    spin_lock_bh(&wr_lock[unit]);
		m = wr_all_packs[unit].ring;
	    spin_unlock_bh(&wr_lock[unit]);
	    if (m <= wr_rings) m=1; else m=0;//буффер на запись в винетик наполовину пуст

	}

	if (l) ret |= POLLIN | POLLRDNORM;
	if (m) ret |= POLLOUT | POLLWRNORM;

    }

    return ret;

}
//***********************************************************
//                       чтение
//************************************************************

static ssize_t rchan_read(struct file *filp, char __user *buff,	size_t count, loff_t *offp)
{
ssize_t ret=0;
unsigned char cmds, bbb, bbs, bbs_last, done, bt, rk, ch1, np, l_ops, l_ips, byte;
unsigned int into, my_tim, dwordik, offsets, ix;
unsigned short st_word, ml_word, wordik, dl, adrest, wrd, word;
int mores, mycount, lp, i, unit, node, l, rdp, wrp, flg=0;
void __iomem *adra=NULL;


    if (count==0) return (atomic_read(&varta));

    node = NODE(filp); unit = UNIT(filp);

    memset(ibuff,0,len_buff);

if (node == Major) {//Altera

    flg=0;
    into=count;  into &= 0xff;	ml_word=into;			// из младшего слова берем команду (мл. байт)
    into=count;  into=(into & 0xffff00)>>8;   st_word=into;	// из старшего и младшего слова берем смещение
    offsets = st_word;
    cmds = ml_word;						// получили байт команды

    switch (cmds) {//анализ заданной команды чтения
	case 1:	
	    ret=0;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		if (subcard[np].type) offsets <<= 4;
		adra = BasedByAdr(offsets);			// чтение 8-ми разрядное
		if (adra!=NULL) {
		    ibuff[0] = ioread8(adra + offsets);
		    ret=1;	flg=1;
		}
	    }
	break;
	case 2://чтение версии прошивки Алтеры на субплате радиоканалов
	    offsets=(offsets<<8)|0xd0; ret=0;
	    bt = AdrToNum(offsets);
	    if (bt<max_card) {
		read_rom_subcard(bt, ibuff, 0);
		ret=strlen(ibuff);
		flg=1;
	    }
	break;
	case 3:	// чтение 8-ми разрядное
	    ret=0;	rk=offsets;	flg=0;
	    if (rk<all_radio) {
		offsets=RK_index[rk] + CS_PiML_AT_COM;
		ix = offsets-0x10;
		if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		adra = BasedByRK(rk);
		if (adra!=NULL) {
		    bbb=(ioread8(adra + ix)) & RDEMP;
		    if (bbb==0) {
			ibuff[0] = ioread8(adra+offsets);
			ret=1;
			flg=1;
		    }
		}
	    }
	break;
	case 0x13:// чтение одного быйта ДЛЯ НОВЫХ ПЛАТ ! ( RXbyte_prn(unsigned char nk, unsigned char prn_bt, unsigned char scr) )
	    ret=0; flg=0; rk=offsets;
	    if (rk<all_radio) {
		if (rk_present[rk]) {
		    adra = BasedByRK(rk);
		    if (adra!=NULL) {
			offsets =  RK_index[rk];	ix = RK_index[rk];
			offsets += CS_PiML_AT_COM;	ix += CS_MIRROR_R_AT;
			if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
			wordik = ioread16(adra+offsets);
			bbb = wordik;//data
			bbs = wordik>>8;
			bbs &= RDEMP;//status
			if (bbs==0) {
			    bt=0; iowrite8(bt,adra+ix);
			    bt=1; iowrite8(bt,adra+ix);
			    bt=0; iowrite8(bt,adra+ix);
			    ibuff[0]=bbb;   ret=1; flg=1;
			}
		    }
		}
	    }
	break;
	case 5://чтение ROM с кросс-платы
	    ret=0;
	break;
	case 4:
 	    my_tim=atomic_read(&varta); 
	    *(unsigned int *)ibuff = my_tim;
	    ret=4;
	    flg=1;
	break;
	case 6:	// чтение всего буфера, конец пакета - только 0x0d
	    ret=0;    mycount=512; 	mores=1;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		ix = offsets-0x10;
		adra = BasedByAdr(ix);
		if (adra!=NULL) {
		    if (subcard[np].type) { offsets <<= 4; ix <<= 4; }
		    bbb=(ioread8(adra+ix)) & RDEMP;
		    if (bbb==0) mores=1; else mores=0;
		    while (mores==1) {
			bbs = ioread8(adra+offsets);
			if ((bbs>0x0a) && (bbs<0x80)) { ibuff[ret]=bbs;   ret++; }
			mycount--; 
			if (bbs==0x0d) mores=0;
			else {
			    bbb=(ioread8(adra+ix)) & RDEMP;
			    if ((bbb==0) && (mycount>0) && (ret<512)) mores=1;
							         else mores=0;
			}
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 7:	// чтение 8-ми разрядное
	    ret=0;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		ix = offsets-0x10;
		adra = BasedByAdr(ix);
		if (subcard[np].type) { offsets <<= 4; ix <<= 4; }
		if (adra!=NULL) {
		    bbb=(ioread8(adra+ix)) & RDEMP;
		    if (bbb==0) {
			ibuff[0] = ioread8(adra+offsets);
			ret=1;
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 8:	// чтение всего буфера, но конец пакета - только 0x0d
	    ret=0;    mycount=512; 	mores=1;
	    rk=offsets;	offsets=RK_index[rk] + CS_PiML_AT_COM;
	    ix = offsets-0x10;
	    adra = BasedByAdr(ix);
	    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
	    if (adra!=NULL) {
		bbb=(ioread8(adra+ix)) & RDEMP;
		if (bbb==0) mores=1; else mores=0;
		while (mores) {
		    bbs = ioread8(adra+offsets);
		    if ((bbs>0x0a) && (bbs<0x80)) { ibuff[ret]=bbs;   ret++; }
		    mycount--;
		    if (bbs==0x0d) mores=0;
		    else {
			bbb=(ioread8(adra+ix)) & RDEMP;
			if ((bbb==0) && (mycount>0) && (ret<512)) mores=1;
						             else mores=0;
		    }
		}
		if (ret>0) flg=1;
	    }
	break;
	case 0x0C:			// чтение всего буфера, почти как case 8, только конец пакета - 0x0a
	    ret=0;    mycount=512; 	mores=1; done=0;
	    rk=offsets;	offsets=RK_index[rk] + CS_PiML_AT_COM;
	    ix = offsets-0x10;
	    adra = BasedByRK(rk);
	    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
	    if (adra!=NULL) {
		bbb=(ioread8(adra+ix)) & RDEMP;
		if (bbb==0) mores=1; else mores=0;
		while (mores) {
		    bbs = ioread8(adra+offsets);
		    if ((bbs>=0x0a) && (bbs<0x80)) {
			if (bbs>=0x20) done=1;
			if (done) { ibuff[ret]=bbs;   ret++; }
		    }
		    bbs_last = bbs;
		    mycount--;
		    if ((bbs==0x0a) && (bbs_last=0x0d)) mores=0;
		    else {
			bbb=(ioread8(adra+ix)) & RDEMP;
			if ((bbb==0) && (mycount>0) && (ret<512)) mores=1; else mores=0;
		    }
		}
		if (ret>0) flg=1;
	    }
	break;
	case 0x1C:			// чтение всего буфера, почти как case 8, только конец пакета - 0x0a + read16
	    ret=0;    mycount=512;	bbs_last=0; done=0;
	    rk=offsets;
	    if (rk<all_radio) {
		adra = BasedByRK(rk);
		if (adra!=NULL) {
		    offsets =  RK_index[rk];	ix = RK_index[rk];
		    offsets += CS_PiML_AT_COM;	ix += CS_MIRROR_R_AT;
		    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		    mores=1;
		    while (mores) {
			wordik = ioread16(adra+offsets);
			mycount--;
			bbb = wordik;//data
			bbs = wordik>>8;
			bbs &= RDEMP;//status
			if (bbs==0) {
			    bt=0; iowrite8(bt,adra+ix);
			    bt=1; iowrite8(bt,adra+ix);
			    bt=0; iowrite8(bt,adra+ix);
			    if ((mycount>0) && (ret<512)) {
				if ((bbb>=0x0a) && (bbb<0x80)) {
				    if (bbb>=0x20) done=1;
				    if (done) {
					ibuff[ret]=bbb;   ret++;
				    }
				}
				//if ((bbb==0x0a) && (done)) mores=0;
				if (bbb==0x0a) mores=0;
			    } else mores=0;
			} else mores=0;
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 0x18:		// чтение всего буфера, но конец пакета - только 0x0d  + read16
	    ret=0;    mycount=512; 	mores=1; done=0;
	    rk=offsets;
	    if (rk<all_radio) {
		adra = BasedByRK(rk);
		if (adra!=NULL) {
		    offsets =  RK_index[rk];	ix = RK_index[rk];
		    offsets += CS_PiML_AT_COM;	ix += CS_MIRROR_R_AT;
		    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		    while (mores) {
			wordik = ioread16(adra+offsets);
			mycount--;
			bbb = wordik;//data
			bbs = wordik>>8;
			bbs &= RDEMP;//status
			if (bbs==0) {
			    //udelay(1);
			    bt=0; iowrite8(bt,adra+ix);
			    bt=1; iowrite8(bt,adra+ix);
			    bt=0; iowrite8(bt,adra+ix);
			    if ((mycount>0) && (ret<512)) {
				if ((bbb>=0x0d) && (bbb<0x80)) {
				    if (bbb>=0x20) done=1;
				    if (done) {
					ibuff[ret]=bbb;   ret++;
				    }
				}
				//if ((bbb==0x0d) && (done)) mores=0;
				if (bbb==0x0d) mores=0;
			    } else mores=0;
			} else mores=0;
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 0x19://0x0d  + read16
	    ret=0;    mycount=1024; 	mores=1; done=0;
	    rk=offsets;
	    if (rk<all_radio) {
		adra = BasedByRK(rk);
		if (adra!=NULL) {
		    offsets =  RK_index[rk];	ix = RK_index[rk];
		    offsets += CS_PiML_AT_COM;	ix += CS_MIRROR_R_AT;
		    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		    while (mores) {
			wordik = ioread16(adra+offsets);
			mycount--;
			bbb = wordik;//data
			bbs = wordik>>8;
			bbs &= RDEMP;//status
			if (bbs==0) {
			    bt=0; iowrite8(bt,adra+ix);
			    bt=1; iowrite8(bt,adra+ix);
			    bt=0; iowrite8(bt,adra+ix);
			    if ((mycount>0) && (ret<1024)) {
				if ((bbb>=0x0d) && (bbb<0x80)) {
				    if (bbb>=0x20) done=1;
				    if (done) {
					ibuff[ret]=bbb;   ret++;
				    }
				}
				if (bbb==0x0d) mores=0;
			    } else mores=0;
			} else mores=0;
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 0x89://0x99//0x0d  + read16
	    ret=0;    mycount=1024; 	mores=1;
	    rk=offsets;
	    if (rk<all_radio) {
		adra = BasedByRK(rk);
		if (adra!=NULL) {
		    offsets =  RK_index[rk];	ix = RK_index[rk];
		    offsets += CS_PiML_AT_COM;	ix += CS_MIRROR_R_AT;
		    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		    while (mores) {
			wordik = ioread16(adra+offsets);
			mycount--;
			bbb = wordik;//data
			bbs = wordik>>8;
			bbs &= RDEMP;//status
			if (bbs==0) {
			    bt=0; iowrite8(bt,adra+ix);
			    bt=1; iowrite8(bt,adra+ix);
			    bt=0; iowrite8(bt,adra+ix);
			    if ((mycount>0) && (ret<1024)) {
				ibuff[ret]=bbb;   ret++;
			    } else mores=0;
			} else mores=0;
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 0x0A:// чтение заданного количества байт из буфера ат_команд 
		// задается значением переменной RK_len_buff[rk], но не более 512 байт
	    ret=0;
	    rk=offsets;
	    if (rk<all_radio) {
		mycount=RK_len_buff[rk];
		if ((mycount<=0) || (mycount>512)) mycount=512;
		offsets=RK_index[rk] + CS_PiML_AT_COM;
		ix = offsets-0x10;
		adra = BasedByRK(rk);
		if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		if (adra!=NULL) {
		    bbb=(ioread8(adra+ix)) & RDEMP;
		    if (!bbb) mores=1; else mores=0;
		    while (mores) {
			bbs = ioread8(adra+offsets);
			ibuff[ret]=bbs;   ret++; 	mycount--;
			if (!mycount) mores=0;
			else {
			    bbb=(ioread8(adra+ix)) & RDEMP;
			    if (!bbb) mores=1; else mores=0;
			}
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;	
	case 0x0B:// чтение заданного количества байт из буфера ат_команд 
		// задается значением переменной RK_len_buff[rk], но не более 2048 байт
	    ret=0;
	    rk=offsets;
	    if (rk<all_radio) {
		mycount=RK_len_buff[rk];
		if ((mycount<=0) || (mycount>2048)) mycount=2048;
		offsets=RK_index[rk] + CS_PiML_AT_COM;
		ix = offsets-0x10;
		adra = BasedByRK(rk);
		if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		if (adra!=NULL) {
		    bbb=(ioread8(adra+ix)) & RDEMP;
		    if (!bbb) mores=1; else mores=0;
		    while (mores) {
			bbs = ioread8(adra+offsets);
			ibuff[ret]=bbs;   ret++; 	mycount--; 
			if (!mycount) mores=0;
			else {
			    bbb=(ioread8(adra+ix)) & RDEMP;
			    if (!bbb) mores=1; else mores=0;
			}
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 0x0F:
	    rk=offsets; ret=0;
	    if (rk<all_radio) ret=atomic_read(&sim900_flag[rk]);
	break;
	case 9://  ЧТЕНИЕ РЕГИСТРА СТАТУСА РК
	    rk=offsets; ret=0;
	    if (rk<all_radio) {
		offsets=RK_index[rk] + CS_PiML_STATUS;
		adra = BasedByRK(rk);
		if (subcard[rk>>2].type) offsets <<= 4;
		if (adra!=NULL) {
		    ibuff[ret] = ioread8(adra+offsets);
		    flg=1; ret++;
		}
	    } else printk(KERN_ALERT "\nKernel k16_read ERROR (command=%d rk=%d)\n", cmds,rk);
	break;
	case 0x10:// чтение 8-ми разрядное с COM2 (IMEI)
	    ret=0;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		adra=BasedByAdr(offsets);
		ix = offsets-0x30;
		if (subcard[np].type) { offsets <<= 4; ix <<= 4; }
		if (adra!=NULL) {
		    bbb=(ioread8(adra+ix)) & RDEMPPROG ;
		    if (bbb==0) {
			ibuff[0] = ioread8(adra+offsets);
			ret=1;
		    }
		    if (ret>0) flg=1;
		}
	    }
	break;
	case 0x81:// чтение 16-ти разрядное
	    ret=0;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		adra=BasedByAdr(offsets);
		if (subcard[np].type) offsets <<= 4;
		if (adra!=NULL) {
		    wordik = ioread16(adra+offsets);
		    memcpy(ibuff,&wordik,2);
		    ret=2;
		    flg=1;
		}
	    }
	break;
	case 0x91:// чтение 16-ти разрядное
	    ret=0;
	    //adra=BasedByAdr(offsets);
	    adra=BaseAdr[0];
	    if (adra!=NULL) {
		wordik = ioread16(adra+offsets);
		memcpy(ibuff,&wordik,2);
		ret=2;
		flg=1;
	    } else printk(KERN_ALERT "\nKernel k16_read ERROR (cmd=0x%X b_adr=NULL adr=0x%X)\n", cmds,offsets);
	break;	
	case 0xA1:// чтение 32-ти разрядное -> 16
	    ret=0;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		adra=BasedByAdr(offsets);
		if (adra!=NULL) {
		    if (subcard[np].type) offsets <<= 4;
		    dwordik = ioread32(adra+offsets);
		    wordik = dwordik;
		    memcpy(ibuff,&wordik,2);
		    ret=2;
		    flg=1;
		} else printk(KERN_ALERT "\nKernel k16_read32 ERROR (cmd=0x%X b_adr=NULL adr=0x%X)\n", cmds,offsets);
	    } else printk(KERN_ALERT "\nKernel k16_read32 ERROR (np=%d cmd=0x%X b_adr=NULL adr=0x%X)\n", np, cmds,offsets);
	break;
	case 0xA2:// чтение 32-ти разрядное -> 16
	    ret=0;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    dwordik = ioread32(adra+offsets);
		    wordik = dwordik;
		    memcpy(ibuff,&wordik,2);
		    ret=2;
		    flg=1;
		} else printk(KERN_ALERT "\nKernel k16_read32 ERROR (np=%d cmd=0x%X b_adr=NULL adr=0x%X)\n", np, cmds,offsets);
	    } else printk(KERN_ALERT "\nKernel k16_read32 ERROR (cmd=0x%X b_adr=NULL adr=0x%X)\n", cmds,offsets);
	break;	
	case 0x8F:// чтение 32-х разрядное
	    ret=0;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		adra=BasedByAdr(offsets);
		if (subcard[np].type) offsets <<= 4;
		if (adra!=NULL) {
		    dwordik = ioread32(adra+offsets);
		    memcpy(ibuff,&dwordik,4);
		    ret=4;
		    flg=1;
		}
	    }
	break;
	case 0x82:// чтение 16-ти разрядное
	    ret=0;
	    offsets <<= 8; offsets |= 0xe0;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    wordik = ioread16(adra+offsets);
		    memcpy(ibuff,&wordik,2);
		    ret=2;
		    flg=1;
		}
	    }
	break;
	case 0x83:// чтение 16-ти разрядное reg_status+data from sim-com-port
	    ret=0;	flg=0;
	    rk=offsets;	
	    if (rk<all_radio) {
		offsets=RK_index[rk] + CS_PiML_SIM_COM;
		adra=BasedByRK(rk);
		if (subcard[rk>>2].type) offsets <<= 4;
		if (adra!=NULL) {
		    wordik = ioread16(adra+offsets);
		    bbb = wordik;//data
		    bbs = (wordik>>8);//status
		    if (bbs & MOD_RDempty) {
			ibuff[0] = bbb;
			ret=1;
			flg=1;
		    }
		}
	    }
	break;
	case 0x84:// ПАКЕТНОЕ чтение 16-ти разрядное reg_status+data from sim-com-port
	    ret=0;	flg=0;
	    rk=offsets;
	    if (rk<all_radio) {
		offsets=RK_index[rk] + CS_PiML_SIM_COM;
		adra=BasedByRK(rk);
		if (subcard[rk>>2].type) offsets <<= 4;
		if (adra!=NULL) {
		    lp=1; mores=255;
		    while (lp) {
			wordik = ioread16(adra+offsets);
			bbb = wordik;//data
			bbs = (wordik>>8);//status
			if (bbs & MOD_RDempty) {
			    ibuff[ret] = bbb;	ret++;
			    flg=1;
			} else lp=0;
			mores--;
			if (!mores) lp=0;
		    }
		}
	    }
	break;
	case 0x85:// чтение 8-и разрядное reg_status+data from sim-com-port
	    ret=0;	flg=0;	wordik=0x20;
	    if (offsets<all_radio) {
		ix = RK_index[offsets] + CS_PiML_SIM_COM;
		adra=BasedByRK(offsets);
		if (subcard[offsets>>2].type) { ix <<= 4; wordik <<= 4; }
		if (adra!=NULL) {
		    bbs = ioread8(adra+ix-wordik);
		    if (bbs & MOD_RDempty) {
			ibuff[0]=ioread8(adra+ix);
			ret=1;
			flg=1;
		    }
		}
	    }
	break;
	case 0x86:// чтение 16-ти разрядное - reg_status+data from sim-com-port
	    ret=0;	flg=0; done=0;
	    rk = offsets;
	    //bbs_last = offsets>>8;
	    if (rk<all_radio) {
		offsets = RK_index[rk] + CS_PiML_SIM_COM;
		ix = RK_index[rk] + CS_MIRROR_R_SIM;
		adra=BasedByRK(rk);
		if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		if (adra!=NULL) {
		    wordik = ioread16(adra+offsets);
		    bbb = wordik;//data
		    bbs = (wordik>>8);//status
		    if (bbs & MOD_RDempty) {
			ibuff[0]=bbb;
			ret=1;
			flg=1;
			bt=0; iowrite8(bt,adra+ix);
			bt=1; iowrite8(bt,adra+ix);
		    }
		}
	    }
	break;
	case 0x88:// блочное 16-ти разрядное чтение
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		i = 0; bt =0;
		dl = len_block_read;
		ret=0; adrest = offsets;
		adra=BasedByAdr(offsets);
		if (adra!=NULL) {
		    if (subcard[np].type) adrest <<= 4;
		    while (i<dl) {
			wordik = ioread16(adra+adrest+bt);
			memcpy(&ibuff[ret],&wordik,2);
			bt += 2; bt &= 0x0f;
			ret+=2;
			i++;
			flg=1;
		    }
		}
	    }
	break;
	case 0x96:// чтение 5-ти байт  16-ти разрядное - reg_status+data from sim-com-port
	    ret=0;	flg=0;
	    rk = offsets;
	    if (rk<all_radio) {
		offsets = RK_index[rk] + CS_PiML_SIM_COM;
		ix = RK_index[rk] + CS_MIRROR_R_SIM;
		adra=BasedByRK(rk);
		if (adra!=NULL) {
		    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		    mycount=5;
		    while (mycount>0) {
			wordik = ioread16(adra+offsets);
			bbb = wordik;//data
			bbs = (wordik>>8);//status
			if (bbs & MOD_RDempty) {
			    ibuff[ret]=bbb;
			    ret++;
			    flg=1;
			    bt=0; iowrite8(bt,adra+ix);
			    bt=1; iowrite8(bt,adra+ix);
			    mycount--;
			} else break;
		    }
		}
	    }
	break;
	case 0x99:
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		if (subcard[np].type) offsets <<= 4;
		bt = ioread8(BaseAdr[0]+offsets);
		ibuff[0]=bt; ret=1; flg=1;
	    }
	break;
	    default : {
		printk(KERN_ALERT "\nKernel k16_read ERROR (unknown command - 0x%X)\n", cmds);
		ret=-1;
	    }

    }

    if (flg) {
	if (copy_to_user(buff,ibuff,ret)) ret=0;
    }

} else if (node == MajorV) {//Vinetic

    if ((unit<0) && (unit>=kol_card)) return 0;

    into=count;
    ml_word=(into & 0x000000ff);	// из младшего слова берем команду (мл. байт)
    st_word=((0x00ffff00 & into)>>8);	// из старшего и младшего слова берем смещение
    offsets = st_word;
    cmds=ml_word;			// получили байт команды

    switch (cmds) {

	case 1:				// чтение 8-ми разрядное
	    ret=0;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) {
			offsets <<= 4;
#ifdef VIN_REG_STATUS
			if (subcard[np].type) {
			    iowrite16((unsigned short)offsets, (void __iomem *)VinAdrAdr[np]);
			    udelay(1);
			}
#endif
		    }
		    ibuff[0]=ioread8(adra+offsets);
		    ret=1;
		    flg=1;
		}
	    }
	break;
	case 8:
	    offsets &= 0x001f;
	    if (offsets<all_radio) {
		wrp=atomic_read(&to_vin[offsets]);
		rdp=atomic_read(&from_vin[offsets]);
		memcpy(ibuff,&wrp,4);
		memcpy(ibuff+4,&rdp,4);
		ret=8;
		flg=1;
	    }
	break;
	case 0x50 :// read check : bxsr1, bxsr2, hwsr1, hwsr2, rd_err, wr_err
		check_dsp(unit);
		mores=atomic_read(&bxsr[unit]);		memcpy(ibuff,&mores,4);
		mores=atomic_read(&hwsr[unit]);		memcpy(ibuff+4,&mores,4);
		mores=atomic_read(&rd_error[unit]);	memcpy(ibuff+8,&mores,4);
		mores=atomic_read(&wr_error[unit]);	memcpy(ibuff+12,&mores,4);
		mores=0; atomic_set(&rd_error[unit],mores); atomic_set(&wr_error[unit],mores);
		flg=1;	ret=16;
	break;
	case 0x20 ://read IPS
	case 0x21 :
	case 0x22 :
	case 0x23 :
	case 0x24 :
	case 0x25 :
	case 0x26 :
	case 0x27 :
	    np=cmds-0x20;
	    if (np<kol_card) {
		spin_lock_bh(&vin_lock[np]);
		    wr_vineticT(0xC100, np, 0);
		    l_ips = rd_vineticT(np,2);
		spin_unlock_bh(&vin_lock[np]);
		ibuff[0] = l_ips;
		flg=1;	ret=1;
	    }
	break;
	case 0x30 ://read OPS
	case 0x31 :
	case 0x32 :
	case 0x33 :
	case 0x34 :
	case 0x35 :
	case 0x36 :
	case 0x37 :
	    np=cmds-0x30;
	    if (np<kol_card) {
		l_ops = atomic_read(&ops[np]);
		ibuff[0] = l_ops;
		flg=1;	ret=1;
	    }
	break;
	case 0x10 ://read pack  with buffer in user level - #define TO_SIP_BUF
	case 0x11 :
	case 0x12 :
	case 0x13 :
	case 0x14 :
	case 0x15 :
	case 0x16 :
	case 0x17 :
	    np=cmds-0x10; ret=4; flg=0;
	    if ((np<kol_card) && (atomic_read(&begin_rtp[np]))) {
		mores=0;	ch1=(np<<2);
		memset(ibuff,0,4);
		spin_lock_bh(&rd_lock[np]);
		    for (l=0; l<4; l++) {
			if ((atomic_read(&ops_list[ch1])) && (atomic_read(&gsm_flag[ch1])>all_radio)) {
			    mores = get_from_rtp_list(ch1, ibuff+ret);
			    if (mores>16) {
				ret+=mores; //flg=1;
				mores>>=1;	byte=mores;
				ibuff[l]=byte;
			    }
			}
			ch1++;
		    }
		spin_unlock_bh(&rd_lock[np]);
		if (ret>4) flg=1; else ret=0;
	    }
	break;
	case 0x60 ://read pack  without buffer in user level - #undef TO_SIP_BUF
	case 0x61 :
	case 0x62 :
	case 0x63 :
	case 0x64 :
	case 0x65 :
	case 0x66 :
	case 0x67 :
	    np=cmds-0x60; ret=4; flg=0;
	    if ((np<kol_card) && (atomic_read(&begin_rtp[np]))) {
		mores=0;	ch1=(np<<2);
		memset(ibuff,0,4);
		spin_lock_bh(&rd_lock[np]);
		    for (l=0; l<4; l++) {
			if (atomic_read(&ops_list[ch1])) {
			    mores = get_from_rtp_list(ch1, ibuff+ret);
			    if (mores>16) {
				ret+=mores;
				mores>>=1;	byte=mores;
				ibuff[l]=byte;
			    }
			}
			ch1++;
		    }
		spin_unlock_bh(&rd_lock[np]);
		if (ret>4) flg=1; else ret=0;
	    }
	break;
	case 0x70 ://read SRE[16]
	case 0x71 :
	case 0x72 :
	case 0x73 :
	case 0x74 :
	case 0x75 :
	case 0x76 :
	case 0x77 :
	    np=cmds-0x70;	ret=0;
	    if (np<kol_card) {
		ret = read_sre(np);//read SRE[16]
		spin_lock_bh(&sre_lock[np]);
		    memcpy(ibuff,&sre_buff[np][0],ret);
		spin_unlock_bh(&sre_lock[np]);
		flg=1;
	    }
	break;
	case 0x40 :
	    np=unit;
	    if (np<kol_card) {
		ch1=offsets;	//indx
		spin_lock_bh(&vin_lock[np]);
		    wrd = rd_vineticT(np,ch1);
		spin_unlock_bh(&vin_lock[np]);
		memcpy(ibuff, &wrd, 2);
		ret=2;
		flg=1;
	    }
	break;
	case 0x81:// чтение только 16-ти разрядное
	    ret=0;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) {
			offsets <<= 4;
#ifdef VIN_REG_STATUS
			iowrite16((unsigned short)offsets, (void __iomem *)VinAdrAdr[np]);
			udelay(1);
#endif
		    }
		    word = ioread16(adra+offsets);
		    memcpy(ibuff, &word, 2);
		    ret=2;
		    flg=1;
		}
	    }
	break;
	    default : {
		printk(KERN_ALERT "\n%s: Kernel read ERROR (unknown command - %X)\n",DevNameV,cmds);
		ret=-1;
	    }
    }

    if ((flg) && (ret>0)) {
	if (ret>len_buff) ret=0;
	else
	if (copy_to_user(buff,ibuff,ret)) {
	    printk(KERN_ALERT "%s: Kernel (copy_to_user) : cmd=%02X, ret=%d\n", DevNameV, cmds, ret);
	    return -EFAULT;
	}
    }

} else {
    printk(KERN_ALERT "\nKernel read ERROR : Major=%d Minor=%d\n", node, unit);
}

    return ret;
}
//***********************************************************
//                         запись
//************************************************************
static ssize_t rchan_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
ssize_t ret=0, counter;
unsigned char bytes0, bytes, bytes_st, rk, rk2, bt, np;
unsigned short word, word1, sho, wordik=0,mycount,uk,slovo,mycount_wr;
unsigned int mmsec, dslovo, param, offsets, ix, offsets_stat, adiks;
int lens, unit, node, i, br, dl, zn, zn2, tdef;
void __iomem *adra=NULL;
unsigned char *uki;
unsigned char mas[4], mas2[4];
char stri[all_radio+1];

    node = NODE(filp); unit = UNIT(filp);

    memset(ibuffw,0,len_buff);

if (node == Major) {//Altera

    ret = count;

    if (copy_from_user(ibuffw,buff,ret)) {
	printk(KERN_ALERT "Kernel k16_write ERROR (copy_from_user) : count=%u\n", ret);
	return -EFAULT;
    }

    bytes0 = *(ibuffw);

    switch (bytes0) {  //анализ принятой команды

	case 1: //команда 1 - write to mem 4000XXXXH 8bit
	    if (count < 4) return -EFAULT;  	// длинна должна быть не менее 4-х байт !!!
	    memcpy(&wordik,ibuffw+1,2);		//offsets
	    offsets=wordik;
	    mycount=count-3;
	    uk=3;			// указатель на 4-й байт - это данные
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    while (mycount>0) {
			bytes=*(ibuffw+uk);
			iowrite8(bytes,adra+offsets);
			mycount=mycount-1;
			uk=uk+1;
		    }
		} else ret=0;
	    } else ret=0;
	break;
	case 0x0E: //Set sim speed
	    if (count < 3) return -EFAULT;  	// длинна должна быть не менее 3-х байт !!!
	    rk = *(ibuffw+1); uk=0;
	    if (rk<all_radio) {
		bytes = *(ibuffw+2);//MirrorData
		offsets = RK_index[rk] + CS_PiML_STATUS;
		adra=BasedByRK(rk);
		if (adra!=NULL) {
		    if (subcard[rk>>2].type) offsets <<= 4;
		    //iowrite8(bytes,BasedByAdr(offsets)+offsets);
		    iowrite8(bytes,adra+offsets);
		    uk=3;
		} else ret=0;
	    }
	break;
	case 0x11: //команда 11 - write to mem 4000XXXXH 8bit
	    if (count != 3) return -EFAULT;  	// длинна должна быть 3 байта !!!
	    rk = *(ibuffw+1);		//nk
	    if (rk<all_radio) {
		offsets=RK_index[rk] + CS_PiML_SIM_COM;
		adra=BasedByRK(rk);
		if (adra!=NULL) {
		    bytes = *(ibuffw+2);	//data
		    ix = RK_index[rk] + CS_MIRROR_W_SIM;
		    if (subcard[rk>>2].type) { offsets <<= 4; ix <<= 4; }
		    iowrite8(bytes,adra+offsets);
		    bt=0; iowrite8(bt,adra+ix);
		    bt=1; iowrite8(bt,adra+ix);
		    ret=1;
		} else ret=0;
	    } else ret=0;
	break;
	case 2: //команда 2 - write one byte to module_sim_data_port
	    if (count != 3) return -EFAULT;
	    rk = *(ibuffw+1);	if (rk>=all_radio) return -EFAULT;
	    offsets_stat=RK_index[rk] + CS_PiML_STATUS;
	    adra=BasedByRK(rk);
	    ret=0;
	    if (adra!=NULL) {
		ix = RK_index[rk] + CS_PiML_SIM_COM;
		if (subcard[rk>>2].type) offsets_stat <<= 4;
		bytes_st=ioread8(adra+offsets_stat);
		if (bytes_st & MOD_WRempty) {
		    bytes=*(ibuffw+2);
		    adra=BasedByAdr(ix);
		    if (subcard[rk>>2].type) ix <<= 4;
		    iowrite8(bytes,adra+ix);
		    ret=1;
		}
	    }
	break;
	case 3: //команда 3 - set mode (with bank or without bank)
	    if (count != 2) return -EFAULT;
	    ret=0;
	    bytes = *(ibuffw+1);
	    offsets=CROSS_BANK_MODE;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite8(bytes,adra+offsets);
		    ret=1;
		}
	    }
	break;
	case 4: //команда 4 - write massiv to module_sim_data_port
	    if (count > 514) return -EFAULT;  	// длинна должна быть не более 514 байт !!!
	    mycount_wr=count-3;
	    rk = *(ibuffw+1);      	// rk
	    bt = *(ibuffw+2);		//ATR flag
	    if (rk>=all_radio) return -EFAULT;
	    offsets=offsets_stat=RK_index[rk];
	    offsets += CS_PiML_SIM_COM;
	    offsets_stat += CS_PiML_STATUS;
	    adra=BasedByRK(rk);
	    ret=0;
	    if (adra!=NULL) {
		if (subcard[rk>>2].type) { offsets <<= 4; offsets_stat <<= 4; }
		if (mycount_wr>0) {
		    uk=3;			// указатель на 3-й байт - это данные
		    while (mycount_wr>0) {
			bytes=*(ibuffw+uk);	// взять старший байт данных
			iowrite8(bytes,adra+offsets);
			mycount_wr--;	uk++;
			ioread8(adra+offsets_stat);
		    }
		    ret=uk-3;
		}
	    }
	break;
	case 0x14: //команда 0x14 - write massiv to module_sim_data_port
	    if (count > 514) return -EFAULT;  	// длинна должна быть не более 514 байт !!!
	    rk = *(ibuffw+1);		// rk
	    if (rk>=all_radio) return -EFAULT;
	    bt = *(ibuffw+2);		//ATR flag
	    mycount_wr=count-3;
	    ret=0;
	    offsets = adiks = RK_index[rk];
	    offsets += CS_PiML_SIM_COM;
	    adiks += CS_MIRROR_ATR;
	    ix = RK_index[rk] + CS_MIRROR_W_SIM;
	    adra=BasedByRK(rk);
	    if (adra!=NULL) {
		if (subcard[rk>>2].type) { offsets <<= 4; adiks <<= 4; ix <<= 4; }
		if (bt) {
		    bt = 0x10; iowrite8(bt,adra+adiks);
		    bt = 0x00; iowrite8(bt,adra+adiks);
		    bt = 0x10; iowrite8(bt,adra+adiks);
		}
		if (mycount_wr>0) {
		    uk=3;			// указатель на 4-й байт - это данные
		    while (mycount_wr>0) {
			bytes=*(ibuffw+uk);	// взять старший байт данных
			iowrite8(bytes,adra+offsets);
			mycount_wr--;	uk++;
			bt=0; iowrite8(bt,adra+ix);
			bt=1; iowrite8(bt,adra+ix);
		    }
		    ret=uk-3;
		}
	    }
	break;
	case 5: //команда 5 - reset timer
	    if (count != 1) return -EFAULT;
	    mmsec=0;	atomic_set(&varta, mmsec);
	    bt = 0x10;
	    for (rk=0; rk<all_radio; rk++) {
		adra=BasedByRK(rk);
		if (adra!=NULL) {
		    adiks = RK_index[rk] + CS_MIRROR_ATR;
		    if (subcard[rk>>2].type) adiks <<= 4;
		    iowrite8(bt,adra+adiks);
		}
	    }
	    ret=1;
	break;
	case 6: //команда 6 - write massiv to module_at_command_port
	    if ((count > 514) || (count<=2)) return -EFAULT;  	// длинна должна быть не более 514 байт !!!
	    mycount_wr=count-2;  
	    rk = *(ibuffw+1);      	// rk
	    if (rk>=all_radio) return -EFAULT;
	    ret=0;
	    adra=BasedByRK(rk);
	    if (adra!=NULL) {
		offsets=RK_index[rk] + CS_PiML_AT_COM;
		uk=2;			// указатель на 3-й байт - это данные
		if (subcard[rk>>2].type) offsets <<= 4;
		while (mycount_wr>0) {
		    bytes=*(ibuffw+uk);	// взять старший байт данных
		    //if (subcard[rk>>2].type) offsets <<= 4;
		    iowrite8(bytes,adra+offsets);
		    mycount_wr--;	uk++;
		}
		ret=uk-2;
	    }
	break;
	case 0x16: //команда 0x16 - write massiv to module_at_command_port (for bit16 mode only)
	    if ((count > 514) || (count<=2)) return -EFAULT;  	// длинна должна быть не более 514 байт !!!
	    mycount_wr=count-2;
	    rk = *(ibuffw+1);      	// rk
	    if (rk>=all_radio) return -EFAULT;
	    ret=0;
	    adra=BasedByRK(rk);
	    if (adra!=NULL) {
		uk=2;			// указатель на 3-й байт - это данные
		offsets=RK_index[rk] + CS_PiML_AT_COM;
		adiks = RK_index[rk] + CS_MIRROR_W_AT;
		if (subcard[rk>>2].type) { offsets <<= 4; adiks <<= 4; }
		while (mycount_wr>0) {
		    bytes=*(ibuffw+uk);	// взять старший байт данных
		    iowrite8(bytes,adra+offsets);
		    mycount_wr--;	uk++;
		    bt=0; iowrite8(bt,adra+adiks);
		    bt=1; iowrite8(bt,adra+adiks);
		}
		ret=uk-2;
	    }
	break;
	case 7: //команда 7 - write massiv (max 512 bytes) to module_at_command_port
	    if (count > 514) mycount_wr=512;  	// длинна должна быть не более 514 байт !!!
			else mycount_wr=count-2;
	    rk = *(ibuffw+1);
	    if (rk>=all_radio) return 0;
	    ret=0;
	    adra=BasedByRK(rk);
	    if (adra!=NULL) {
		uk=2;// указатель на 3-й байт - это данные
		offsets      = RK_index[rk] + CS_PiML_AT_COM;
		offsets_stat = RK_index[rk] + CS_PiML_STATUS;
		if (subcard[rk>>2].type) { offsets <<= 4; offsets_stat <<= 4; }
		while (mycount_wr>0) {
		    if ((ioread8(adra+offsets_stat)) & WREMP) {
			bytes=*(ibuffw+uk);	// взять старший байт данных
			iowrite8(bytes,adra+offsets);
			mycount_wr--;	uk++;
		    }
		    ret=uk-2;
		}
	    }
	break;
	case 0x0F: //команда 0x0B - write massiv (max 512 bytes) to module_at_command_port in the timer
	    if (count > 2050) lens=2048;  	// длинна должна быть не более 2048 байт !!!
			 else lens=count-2;
	    rk = *(ibuffw+1);
	    if (rk>=all_radio) return 0;
	    ret=0;
	    atomic_set(&sim900_type[rk], 1);
	    if (atomic_read(&sim900_flag[rk])) {
		memcpy(&sim900_buf[rk][0], ibuffw+2, lens);
		atomic_set(&sim900_len[rk], lens);
		atomic_set(&sim900_txd[rk], ret);
		atomic_set(&sim900_flag[rk], ret);
		ret=lens;
	    }
	break;
	case 9: //команда 9 - write massiv (max 2048 bytes) to module_at_command_port
	    if (count > 2050) mycount_wr=2050; 	// длинна должна быть не более 2050 байт !!!
			else mycount_wr=count-2;  
	    rk = *(ibuffw+1);	
	    if (rk>=all_radio) return 0;
	    ret=0;
	    adra=BasedByRK(rk);
	    if (adra!=NULL) {
		uk=2;// указатель на 3-й байт - это данные
		offsets      = RK_index[rk] + CS_PiML_AT_COM;
		offsets_stat = RK_index[rk] + CS_PiML_STATUS;
		if (subcard[rk>>2].type) { offsets <<= 4; offsets_stat <<= 4; }
		while (mycount_wr>0) {
		    if ((ioread8(adra+offsets_stat)) & WREMP) {
			bytes=*(ibuffw+uk);	// взять старший байт данных
			iowrite8(bytes,adra+offsets);
			mycount_wr--;	uk++;
		    }
		    ret=uk-2;
		}
	    }
	break;
	case 8: //команда 8 - set len for read_0A_0B_commands
	    if (count != 6) return -EFAULT;
	    rk = *(ibuffw+1);
	    if (rk>=all_radio) ret=0;
	    else {
		memcpy(&RK_len_buff[rk], ibuffw+2, 4);
		ret=6;
	    }
	break;
	case 0x0A: //команда 0x0a - write 1 byte to module_at_command_port
	    if (count != 3) return 0;
	    rk = *(ibuffw+1);
	    if (rk>=all_radio) return 0;
	    ret=0;
	    adra=BasedByRK(rk);
	    if (adra!=NULL) {
		offsets      = RK_index[rk] + CS_PiML_AT_COM;
		offsets_stat = RK_index[rk] + CS_PiML_STATUS;
		if (subcard[rk>>2].type) { offsets <<= 4; offsets_stat <<= 4; }
		if ((ioread8(adra+offsets_stat)) & WREMP) {
		    bytes=*(ibuffw+2);
		    iowrite8(bytes,adra+offsets);
		    ret=1;
		}
	    }
	break;
	case 0x1A: //команда 0x1a - write 1 byte to module_at_command_port (for bit16 mode only)
	    if (count != 3) return 0;
	    rk = *(ibuffw+1);
	    if (rk>=all_radio) return 0;
	    ret=0;
	    adra=BasedByRK(rk);
	    if (adra!=NULL) {
		offsets      = RK_index[rk] + CS_PiML_AT_COM;
		offsets_stat = RK_index[rk] + CS_PiML_STATUS;
		adiks 	     = RK_index[rk] + CS_MIRROR_W_AT;
		if (subcard[rk>>2].type) { offsets <<= 4; offsets_stat <<= 4; adiks <<= 4; }
		if ((ioread8(adra+offsets_stat)) & WREMP) {
		    bytes=*(ibuffw+2);
		    iowrite8(bytes,adra+offsets);
		    ret=1;
		    bt=0; iowrite8(bt,adra+adiks);
		    bt=1; iowrite8(bt,adra+adiks);
		}
	    }
	break;
	case 0x0B: //get from user_level "rk_present[all_radio]"
	    if (count != all_radio+1) return -EFAULT;
	    memcpy(&rk_present[0], ibuffw+1, all_radio);
	    bt=0; rk=0;
	    while (bt<max_card) {
		if (rk_present[rk]) subboard[bt]=1;
		rk += 4;
		bt++;
	    }
	    ret=all_radio;
	    memset(stri,0,all_radio+1); for (bt=0; bt<kol_card; bt++) sprintf(stri+strlen(stri),"%d",subboard[bt]);
	    printk(KERN_ALERT "%s: subboards : %s\n", DevName, stri);
	    memset(stri,0,all_radio+1); for (bt=0; bt<all_radio; bt++) sprintf(stri+strlen(stri),"%d",rk_present[bt]);
	    printk(KERN_ALERT "%s: channels  : %s\n", DevName, stri);
	break;
	case 0x0C: //set G8_ONLY mode		//!!!!!!!!!!!!  ???????????????????????   !!!!!!!!!!!!!!!!!
	    ret=0;
/*
	    if (count != 2) return -EFAULT;  	// длинна должна быть 2 байта !!!
	    bytes = *(ibuffw+1);      		// data
	    offsets = (unsigned short)G8_ONLY;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		iowrite8(bytes,adra+offsets);
		ret=2;
	    }
*/
	break;
	case 0x81: //команда 81H - write to mem 4000XXXXH 16bit
	    memcpy(&word,ibuffw+1,2);//адрес
	    offsets = word;
	    memcpy(&slovo,ibuffw+3,2);//данные
	    ret=0;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite16(slovo,adra+offsets);
		    ret=2;
		}
	    }
	break;
	case 0x91: //команда 91H - write to mem 16bit
	    memcpy(&word,ibuffw+1,2);//адрес
	    offsets = word;
	    memcpy(&slovo,ibuffw+3,2);//данные
	    ret=0;
	    //adra=BasedByAdr(offsets);
	    adra=BaseAdr[0];
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite16(slovo,adra+offsets);
		    ret=2;
		}
	    } else printk(KERN_ALERT "\nKernel k16_write ERROR (cmd=0x%X b_adr=NULL adr=0x%X)\n", bytes0,offsets);
	break;
	case 0xA1: //команда A1H - write to mem 32bit
	    memcpy(&word,ibuffw+1,2);//адрес
	    memcpy(&slovo,ibuffw+3,2);//данные
	    dslovo = slovo;
	    offsets = word;
	    ret=0;
	    np = AdrToNum(offsets);
	    if (np<max_card) {
		adra=BasedByAdr(offsets);
		if (adra!=NULL) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite32(dslovo,adra+offsets);
		    ret=2;
		} else printk(KERN_ALERT "\nKernel k16_write32 ERROR (np=%d cmd=0x%X b_adr=NULL adr=0x%X)\n", np, bytes0,offsets);
	    } else printk(KERN_ALERT "\nKernel k16_write32 ERROR (cmd=0x%X b_adr=NULL adr=0x%X)\n", bytes0,offsets);
	break;
	case 0xA2: //команда A1H - write to mem 32bit
	    memcpy(&word,ibuffw+1,2);//адрес
	    memcpy(&slovo,ibuffw+3,2);//данные
	    dslovo = slovo;
	    offsets = word;
	    ret=0;
	    adra=BasedByAdr(offsets);
	    //adra=BaseAdr[0];
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite32(dslovo,adra+offsets);
		    ret=2;
		} else printk(KERN_ALERT "\nKernel k16_write32 ERROR (np=%d cmd=0x%X b_adr=NULL adr=0x%X)\n", np, bytes0,offsets);
	    } else printk(KERN_ALERT "\nKernel k16_write32 ERROR (cmd=0x%X b_adr=NULL adr=0x%X)\n", bytes0,offsets);
	break;
	case 0x8F: //команда 8FH - write to mem 4000XXXXH 32bit
	    if (count < 7) return -EFAULT;  	// длинна должна быть не менее 6-х байт !!!
	    memcpy(&word,ibuffw+1,2);//адрес
	    offsets = word;
	    memcpy(&dslovo,ibuffw+3,4);//данные
	    ret=0;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite32(dslovo,adra+offsets);
		    ret=4;
		}
	    }
	break;
	case 0x87: //команда 87H - запись длинны для команды чтения блока
	    if (count < 2) return -EFAULT;  	// длинна должна быть не менее 2-х байт !!!
	    len_block_read = *(ibuffw+1);
	    ret=2;
	break;
	case 0x88: //команда 88H - write to mem 4000XXXXH 16bit block
	    if (count < 6) return -EFAULT;  	// длинна должна быть не менее 6-ти байт !!!
	    dl=*(ibuffw+1);//длинна блока в словах (16bit)
	    memcpy(&word,ibuffw+2,2);//адрес
	    offsets = word;
	    adiks = offsets & 0xff00;
	    uk = 4;
	    ret=0;
	    bt=0;
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) { offsets <<= 4; adiks <<= 4; }
		    while (ret<dl) {
			ioread8(adra+adiks);
			memcpy(&slovo,ibuffw+uk,2);//данные
			iowrite16(slovo,adra+offsets+bt);
			bt += 2; bt &= 0x0f;
			uk += 2;
			ret++;
		    }
		    ret=ret<1;
		}
	    }
	break;
	case 0x0D: //set commut_KT
	    if (count != 2) return -EFAULT;  	// длинна должна быть 2 байта !!!
	    rk = *(ibuffw+1);      		// data
	    slovo=0; dl=0;
	    for (bt=0; bt<max_card; bt++) {
		if (subboard[bt]) {
		    ix = (unsigned short)KT_POINT+slovo;
		    adra=BasedByAdr(ix);
		    if (adra!=NULL) {
			if (subcard[bt].type) ix <<= 4;
			iowrite8(rk,adra+ix);
		    }
		    dl++;
		}
		slovo+=0x0200;
	    }
	    printk(KERN_ALERT "%s: Set PointKT for channel %d.\n",DevName,rk);
	    ret=dl;
	break;
	case 0xFF: //set tick_mode for timer (0- 2mcek, 1- 1msec)
	    if (count != 2) return -EFAULT;  	// длинна должна быть 2 байта !!!
	    bt =*(ibuffw+1);// data : timi_def=5 or 10
	    if (bt) tdef=10; else tdef=5;
	    atomic_set(&tick_mode,tdef);
	    ret=2;
	    printk(KERN_ALERT "%s: Set timer mode to %d msec.\n",DevName,10/tdef);
	break;
	case 0x10://SET/UNSET DSS_ONLY (1-set , 0-unset)
	    if (count != 2) return -EFAULT;
	    bt =*(ibuffw+1);// data
	    atomic_set(&dss_only,(int)bt);
	    ret=2;
	break;
	    default : {
		printk(KERN_ALERT "\nKernel k16_write ERROR (unknown command - 0x%X)\n", bytes0);
		ret=-1;
	    }

    }

} else if (node == MajorV) {//Vinetic

    if ((unit<0) || (unit>=kol_card)) return 0;

    counter=count;	if (counter>len_buff) counter=len_buff;

    if (copy_from_user(ibuffw,buff,counter)) {
	printk(KERN_ALERT "%s: Kernel write ERROR (copy_from_user) : counter=%d\n", DevNameV,counter);
	return -EFAULT;
    }

    ret=counter;

    bytes0 = *(unsigned char *)(ibuffw);

    switch (bytes0) {//анализ принятой команды

	case 1: //команда 1 - write to mem 4000XXXXH 8bit
	    if (counter < 4) return -EFAULT;  	// длинна должна быть не менее 4-х байт !!!
	    //bytes = *(ibuffw+1);      	// low byte offsets
	    //bytes_st = *(ibuffw+2);   	// high byte offsets
	    //word = bytes_st; 
	    //offsets = ((word<<8)|bytes);// формируем адрес смещения из 2-го и 3-го байтов
	    memcpy(&word,ibuffw+1,2);
	    offsets = word;
	    bytes=*(unsigned char *)(ibuffw+3);// указатель на 4-й байт - это данные
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<max_card) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite8(bytes,adra+offsets);
		    ret=1;
		} else ret=0;
	    } else ret=0;
	break;
	case 2 ://Установить флаг проверки винетиков (начать проверку на наличие пакетов)
	    if (counter != 2) return -EFAULT;
	    br=1;
	    np=*(ibuffw+1);//np
	    if (np>=kol_card) {
		for (i=0; i<kol_card; i++) {
		    atomic_set(&rd_error[i],0);
		    atomic_set(&wr_error[i],0);
		    spin_lock_bh(&rd_lock[i]);
			for (rk=0; rk<4; rk++) clr_all_packs((i<<2)+rk);
		    spin_unlock_bh(&rd_lock[i]);
		    spin_lock_bh(&wr_lock[i]);
			wr_clr_all_packs(i);
		    spin_unlock_bh(&wr_lock[i]);
		    atomic_set(&begin_rtp[i],br);
		}
	    } else {
		atomic_set(&rd_error[np],0);
		atomic_set(&wr_error[np],0);
		spin_lock_bh(&rd_lock[np]);
		    for (rk=0; rk<4; rk++) clr_all_packs((np<<2)+rk);
		spin_unlock_bh(&rd_lock[np]);
		spin_lock_bh(&wr_lock[np]);
		    wr_clr_all_packs(np);
		spin_unlock_bh(&wr_lock[np]);
		atomic_set(&begin_rtp[np],br);
	    }
	    ret=1;
	break;
	case 3 ://Сбросить флаг проверки винетиков (отменить проверку на наличие пакетов)
	    if (counter != 2) return -EFAULT;
	    br=0;
	    np=*(ibuffw+1);//np
	    if (np>=kol_card) {
		for (i=0; i<kol_card; i++) {
		    atomic_set(&begin_rtp[i],br);
		    atomic_set(&rd_error[i],0);
		    atomic_set(&wr_error[i],0);
		    spin_lock_bh(&wr_lock[i]);
			wr_clr_all_packs(i);
		    spin_unlock_bh(&wr_lock[i]);
		    spin_lock_bh(&rd_lock[i]);
			for (rk=0; rk<4; rk++) clr_all_packs((i<<2)+rk);
		    spin_unlock_bh(&rd_lock[i]);
		}
	    } else {
		atomic_set(&begin_rtp[np],br);
		spin_lock_bh(&wr_lock[np]);
		    wr_clr_all_packs(np);
		spin_unlock_bh(&wr_lock[np]);
		spin_lock_bh(&rd_lock[np]);
		    for (rk=0; rk<4; rk++) clr_all_packs((np<<2)+rk);
		spin_unlock_bh(&rd_lock[np]);
		atomic_set(&rd_error[np],0); 
		atomic_set(&wr_error[np],0);
	    }
	    ret=1;
	break;
	case 4:// принять массив unsigned char subboard[kol_card]
	    i = counter-1;
	    if ((i<4) || (i>kol_card)) return -EFAULT;
	    memcpy(subboard, ibuffw+1, i);
	    subboard_flag=1;
	    ret=counter;//5 или 7
	break;
	case 5:// SET DSS_ONLY_MODE 
	    if (counter != 1) return -EFAULT;
	    br=1;	atomic_set(&dss_only,br);
	    ret=1;
	break;
	case 6:// UNSET DSS_ONLY_MODE 
	    if (counter != 1) return -EFAULT;
	    br=0;	atomic_set(&dss_only,br);
	    ret=1;
	break;
	case 7 ://"set_coder_conf_RTP + set_coder_chan_conf_RTP"
	    if ((counter < 3) || (counter > 4)) return -EFAULT;
	    if (counter == 3) bt=1;//ALM
	    else 
	    if (counter == 4) bt=*(unsigned char *)(ibuffw+3);//ALM(0x11) or PCM(0x01) resource code 1=ALM, 0=PCM
	    np=*(ibuffw+1);//np
	    rk=*(ibuffw+2);//nk in vin 0..3
	    if ((np<kol_card) && (rk<4)) {
		bytes = (np << 2) + (rk&3);
		spin_lock_bh(&rd_lock[np]);
		    clr_all_packs(bytes);
		spin_unlock_bh(&rd_lock[np]);
		atomic_set(&rtp_init[bytes], 0);
		atomic_set(&rtp_init2[bytes], 0);
		atomic_set(&rk_rtp[np][rk], 1);
		if (start_all_rtp(np, rk, bt)) ret=4; else ret=0;
	    } else ret=0;
	break;
	case 12 ://"near_echo"
	    if (counter != 7) return -EFAULT;
	    np=*(ibuffw+1);//np
	    rk=*(ibuffw+2);//nk in vin 0..3
	    sho=*(unsigned short *)(ibuffw+3);//first param
	    word=*(unsigned short *)(ibuffw+5);//second param
	    if (np<kol_card) {
		param=sho;	param<<=16;	param|=word;
		atomic_set(&near_echo_param[np][rk], param);
		atomic_set(&set_near_echo_param[np][rk], 1); 
		if (do_near_echo(np, rk)) ret=7; else ret=0;
	    } else ret=0;
	break;
	case 13 ://"far_echo"
	    if (counter != 7) return -EFAULT;
	    np=*(ibuffw+1);//np
	    rk=*(ibuffw+2);//nk in vin 0..3
	    sho=*(unsigned short *)(ibuffw+3);//first param
	    word=*(unsigned short *)(ibuffw+5);//second param
	    if (np<kol_card) {
		param=sho;	param<<=16;	param|=word;
		atomic_set(&far_echo_param[np][rk], param);
		atomic_set(&set_far_echo_param[np][rk], 1);
		if (do_far_echo(np, rk)) ret=7; else ret=0;
	    } else ret=0;
	break;
	case 14 ://"done_rtp"
	    if (counter != 3) return -EFAULT;
	    np=*(ibuffw+1);//np
	    rk=*(ibuffw+2);//nk in vin 0..3
	    if (np<kol_card) {
		bytes = (np << 2) + (rk&3);
		spin_lock_bh(&rd_lock[np]);
		    clr_all_packs(bytes);
		spin_unlock_bh(&rd_lock[np]);
		atomic_set(&done_rtp[np][rk], 1);
		if (do_done_rtp(np, rk)) ret=3; else ret=0;
	    } else ret=0;
	break;
	case 15 ://case 11 ://"set_param_rtp"  +  case 8 ://"set_signaling_RTP"
	    if ((counter < 9) || (counter > 10)) return -EFAULT;
	    if (counter == 9) bt=1;//ALM
	    else 
	    if (counter == 10) bt=*(unsigned char *)(ibuffw+9);//ALM(0x11) or PCM(0x01) resource code 1=ALM, 0=PCM
	    np=*(ibuffw+1);//np
	    rk=*(ibuffw+2);//nk in vin 0..3
	    sho=*(unsigned short *)(ibuffw+3);//first param	- VAD+CN
	    word=*(unsigned short *)(ibuffw+5);//second param	- GAIN
	    word1=*(unsigned short *)(ibuffw+7);//SIP_tabl[ch].evt_payload;
	    if (np<kol_card) {
	    	param=sho;	param<<=16;	param|=word;
		atomic_set(&param_rtp[np][rk], param);//VAD+CN and GAIN
		atomic_set(&set_mix_param_rtp[np][rk], 1);
		atomic_set(&evt_rtp[np][rk], (int)word1);
		atomic_set(&set_mix2_param_rtp[np][rk], 1); 
		if (init_all_mix_param_rtp(np, rk, bt)) ret=10; else ret=0;
	    } else ret=0;
	break;
	case 16 ://RESET vinetic :  set 0 (begin reset)
	    if (counter!=1) return -EFAULT;
	    word = unit; ret=0;
	    if (word<kol_card) {
		bytes = 0;      // data for write
		offsets=(word<<9) + CS_RESET_VINETIC;
		adra=BasedByAdr(offsets);
		if (adra!=NULL) {
		    if (subcard[word].type) offsets <<= 4;
		    iowrite8(bytes,adra+offsets);
		    ret=1;
		}
	    }
	break;
	case 17 ://RESET vinetic :  set 1 (end reset)
	    if (counter!=1) return -EFAULT;
	    word = unit; ret=0;
	    if (word<kol_card) {
		bytes = 1;      // data for write
		offsets=(word<<9) + CS_RESET_VINETIC;
		adra=BasedByAdr(offsets);
		if (adra!=NULL) {
		    if (subcard[word].type) offsets <<= 4;
		    iowrite8(bytes,adra+offsets);
		    ret=1;
		}
	    }
	break;
	case 18 ://425_Hz to GSM  on/off
	    if (counter!=3) return -EFAULT;
	    np = unit;	ret=0;
	    if (np<kol_card) {
		rk=*(ibuffw+1);//nk in vin 0..3
		if (rk<4) {
		    bytes = *(ibuffw+2);//
		    if (do_425(np,rk,bytes)) ret=3;
		}
	    }
	break;
	case 19 ://set_gsm_gsm
	    if (counter!=3) return -EFAULT;
	    ret=0;	
	    rk = *(ibuffw+1);//номер 1-го рк 0..31
	    rk2 = *(ibuffw+2);//номер 2-го рк 0..31
	    if ((rk<all_radio) && (rk2<all_radio)) {
		atomic_set(&to_vin[rk], 0);	atomic_set(&to_vin[rk2], 0);
		atomic_set(&from_vin[rk], 0);	atomic_set(&from_vin[rk2], 0);
		zn = rk2;
		zn2 = rk;
		atomic_set(&gsm_flag[rk], zn);
		atomic_set(&gsm_flag[rk2], zn2);
		ret=3;
	    }
	break;
	case 20 ://unset_gsm_gsm
	    if (counter!=3) return -EFAULT;
	    ret=0;	
	    rk = *(ibuffw+1);//номер 1-го рк 0..31
	    rk2 = *(ibuffw+2);//номер 2-го рк 0..31
	    if ((rk<all_radio) && (rk2<all_radio)) {
		zn = 255;
		atomic_set(&gsm_flag[rk], zn);
		atomic_set(&gsm_flag[rk2], zn);
		ret=3;
	    }
	break;
	case 21 ://full reset vinetic
	    if (counter!=1) return -EFAULT;
	    np = unit; ret=0;
	    if (np<kol_card) {
		reset_vinetic(np);
		ret=1;
	    }
	break;
	case 0x81: //команда 81H - write to mem 4000XXXXH 16bit
	    if (counter < 5) return -EFAULT;  	// длинна должна быть не менее 5-ти байт !!!
	    ret=0;
	    memcpy(&word,ibuffw+1,2);//адрес
	    offsets = word;
	    memcpy(&slovo,ibuffw+3,2);//данные
	    adra=BasedByAdr(offsets);
	    if (adra!=NULL) {
		np = AdrToNum(offsets);
		if (np<kol_card) {
		    if (subcard[np].type) offsets <<= 4;
		    iowrite16(slovo,adra+offsets);
		    ret=2;
		}
	    }
	break;
	case 0x20: //команда 20H - write to mem 4000XXXXH 16bit   -  write packet
	case 0x21:
	case 0x22:
	case 0x23:
	case 0x24:
	case 0x25:
	case 0x26:
	case 0x27:
	    np=bytes0-0x20; 
	    ret=0;
	    if ((atomic_read(&begin_rtp[np])) && (np<kol_card)) {
		memset(mas,0,4);	memset(mas2,0,4);
		uki=ibuffw+6;//+2
		memcpy(mas,ibuffw+2,4);
		word=counter-6;//-2
		if (word) {
		    for (rk=0; rk<4; rk++) {
			dl=mas[rk];
			if (dl) {
			    dl<<=1;
			    spin_lock_bh(&wr_lock[np]);
				wr_put_to_rtp_list(np, uki, dl);
				atomic_set(&ips_list[np], 1);
			    spin_unlock_bh(&wr_lock[np]);
			    uki+=dl;
			    mas2[rk]=mas[rk];
			}
		    }
		    memcpy(&ret,mas2,4);
		}
	    }
	break;
	case 0x40 :
	    if (counter < 4) return -EFAULT;  	// длинна должна быть не менее 4-х байт !!!
	    np = unit;
	    if (np<kol_card) {
		bytes_st = *(ibuffw+1);   	// indx - смещение
		memcpy(&slovo,ibuffw+2,2);
		spin_lock_bh(&vin_lock[np]);
		    wr_vineticT(slovo, np, bytes_st);
		spin_unlock_bh(&vin_lock[np]);
		ret=2;
	    } else ret=0;
	break;
	    default : {
		printk(KERN_ALERT "\n%s: Kernel write ERROR (unknown command - %X)\n", DevNameV, bytes0);
		ret=-1;	
	    }
    }


} else {
    printk(KERN_ALERT "\nKernel write ERROR : Major=%d Minor=%d\n", node, unit);
}

    return ret;

}
//************************************************************
//*************************************************************
//*************************************************************
//*************************************************************
static struct file_operations rchan_fops = {
  .owner   = THIS_MODULE,
  .open    = rchan_open,
  .release = rchan_release,
  .read    = rchan_read,
  .write   = rchan_write,
  .poll    = rchan_poll,
};
//************************************************************
//
//************************************************************
static void init_sio(struct rchan_sio *sio)
{
  dev_t dev = MKDEV(Major,0);//for Altera
  dev_t devV = MKDEV(MajorV,0);//for Vinetic

  cdev_init(&sio->cdev, &rchan_fops);//for Altera
  cdev_init(&sio->cdevV, &rchan_fops);//for Vinetic

  cdev_add(&sio->cdev, dev, 1);//for Altera
  cdev_add(&sio->cdevV, devV, kol_card);//for Vinetic

}
//************************************************************
//		    
//************************************************************
static void deinit_sio(struct rchan_sio *sio) {

    cdev_del(&sio->cdev);//for Altera
    cdev_del(&sio->cdevV);//for Vinetic
}

static struct rchan_sio chan_sio;
//************************************************************
//************************************************************
//************************************************************
//************************************************************
//************************************************************
// ************************************************************
//              init devices
// ************************************************************
static int __init rchan_init(void)
{

dev_t dev;
dev_t devV;
int i=0, k, l, j, none=255, rc=0, err, present=0;
unsigned char bt, bbb, sbrd;//, n;
unsigned short word;
unsigned int ixa, ixb, ixc;
char st[16];
char stri[all_radio+1];

    printk(KERN_ALERT "\n");
    memset(&rk_present[0],0,all_radio);
    memset(&subboard[0],0,max_card);
    bt=0x11; sbrd=0x10; bbb=0;
    for (i=0; i<max_card; i++) {
	subcard[i].BaseAdrRAM = NULL;
	subcard[i].num = bbb;
	subcard[i].adr = bt;
	subcard[i].adr10 = sbrd;
	subcard[i].id = 0xffff;
	subcard[i].type = 0;
	bt+=2; sbrd+=2; bbb++;
    }
//----------------------------------------------------------------------------
    tcard=0;
    while (tcard<max_brd) {
	gsm_board[tcard]=NULL;
	BaseAdr[tcard]=NULL;

	if (tcard>0) {
	    if (gsm_board[tcard-1]!=NULL)
		gsm_board[tcard] = pci_get_device(0xDEAD, PCI_ANY_ID, gsm_board[tcard-1]);
	} else {
	    gsm_board[tcard] = pci_get_device(0xDEAD, PCI_ANY_ID, NULL);
	    if (gsm_board[tcard]==NULL) break;
	}

	if (gsm_board[tcard]==NULL) break;

	err=0;
	err = pci_enable_device(gsm_board[tcard]);
	if (err) { 
	    gsm_board[tcard]=NULL;
	    continue;
	}
	//printk(KERN_ALERT "%s: board %d : PCI board was found.\n",DevName,tcard+1);
	printk(KERN_ALERT "%s: PCI board %d was found => vendor:device=0x%X:0x%X\n",
		    DevName, tcard+1, gsm_board[tcard]->vendor, gsm_board[tcard]->device);
	
	//err = pci_request_region(gsm_board[tcard], 1, "root");
	err = pci_request_regions(gsm_board[tcard], "k16_mod");
	if (err) {
	    printk(KERN_ALERT "%s: board %d : PCI board request regions ERROR!\n",DevName,tcard+1);
	    pci_disable_device(gsm_board[tcard]);
	    gsm_board[tcard]=NULL;
	}
	if (!err) present=1;
	     else break;
	if (gsm_board[tcard]!=NULL) {
	    for (k=0; k<bar_count; k++) {
		bar_start[tcard][k] = pci_resource_start(gsm_board[tcard], k);
		bar_len[tcard][k]   = pci_resource_len(gsm_board[tcard], k);
		bar_flag[tcard][k]  = pci_resource_flags(gsm_board[tcard], k);
		printk(KERN_ALERT "%s:\tboard %d BAR %d: start=0x%X len=0x%X flag=0x%X\n",
					DevName,tcard+1,k,
					(unsigned int)bar_start[tcard][k],
					(unsigned int)bar_len[tcard][k],
					(unsigned int)bar_flag[tcard][k]);
		if (bar_len[tcard][k]>0x10000) {//65536
		    if (bar_start[tcard][k]) {
			if (!(BaseAdr[tcard] = ioremap_nocache(bar_start[tcard][k],0x100000))) {//100000=1M//0x2000=8192
			    printk(KERN_ALERT "%s: board %d : can't remap i/o memory\n",DevName,tcard+1);
			    goto err_out;
			}
			printk(KERN_ALERT "%s: board %d : Mapping 0x%X to memory_addr 0x%08X\n",
				DevName,tcard+1,(unsigned int)bar_start[tcard][k],(unsigned int)BaseAdr[tcard]);
			//----------------------------------------------------------------------
			for (l=0; l<max_card; l++) {
			    if (subcard[l].id == 0xffff) {
				ixa = subcard[l].adr; ixa <<= 8;
				ixa |= (CS_PRESENCE & 0x00ff);
				if (gsm_board[tcard]->device == 0xbede) ixa <<= 4;//PCIe
				word = ioread16(BaseAdr[tcard]+ixa);
				//printk(KERN_ALERT "%s: board %d : addr=0x%08X data=0x%04X\n",DevName,tcard+1,(unsigned int)BaseAdr[tcard]+ixa,word);
				if ((word&msk_id)==0x0900) {
				    printk(KERN_ALERT "%s: board %d : addr=0x%08X data=0x%04X\n",
						DevName,tcard+1,(unsigned int)BaseAdr[tcard]+ixa,word);
				    sbrd = subcard[l].num;
				    subboard[sbrd]=1;
				    subcard[sbrd].BaseAdrRAM=BaseAdr[tcard];
				    subcard[sbrd].id = word;
				    if (gsm_board[tcard]->device == 0xbede) subcard[sbrd].type = 1;//PCIe
				    for (j=0; j<4; j++) rk_present[(sbrd<<2)+j]=1;
				}
			    }
			}
			//----------------------------------------------------------------------
			break;
		    }
		}
	    }
	}//if (gsm_board[tcard]!=NULL)
	tcard++;
    }
    if (!present) {
	printk(KERN_ALERT "%s: Polygator's PCI boards was NOT found.\n",DevName);
	return 1;
    }

/*------------------------------------------------------------*/

    if (!Major) {
	if ((alloc_chrdev_region(&dev, 0, 1, DevName)) < 0) {
	    printk(KERN_ALERT "%s: Allocation device failed\n",DevName);
	    return 1;
	}
	Major = MAJOR(dev);
	printk(KERN_ALERT "%s: Device allocated with major number %d \n",DevName,Major);
    } else {
	if (register_chrdev_region(MKDEV(Major,0),1,DevName)<0) {
	    printk(KERN_ALERT "%s: Registration failed\n",DevName);
	    return 1;
	}
	printk(KERN_ALERT "%s: Device registered\n",DevName);
    }

    if (!MajorV) {
	if ((alloc_chrdev_region(&devV, 0, kol_card, DevNameV)) < 0) {
	    printk(KERN_ALERT "%s: Allocation device failed\n",DevNameV);
	    return 1;
	}
	MajorV = MAJOR(devV);
#ifdef VIN_REG_STATUS
	printk(KERN_ALERT "%s: %d Device allocated with major number %d (register)\n",DevNameV,kol_card,MajorV);
#else
	printk(KERN_ALERT "%s: %d Device allocated with major number %d (pin)\n",DevNameV,kol_card,MajorV);
#endif
    } else {
	if (register_chrdev_region(MKDEV(MajorV,0),kol_card,DevNameV)<0) {
	    printk(KERN_ALERT "%s: Registration failed\n",DevNameV);
	    return 1;
	}
	printk(KERN_ALERT "%s: Device registered\n",DevNameV);
    }

    init_sio(&chan_sio);

    altera_class = class_create(THIS_MODULE, "cyclon");
    CLASS_DEV_CREATE(altera_class, MKDEV(Major, 0), NULL, DevName);

    vin_class = class_create(THIS_MODULE, "vinetic");
    for (i=0; i<kol_card; i++) {
	memset(st,0,16);	sprintf(st,"vin%d",i);
	CLASS_DEV_CREATE(vin_class, MKDEV(MajorV, i), NULL, st);
    }
    my_dev_ready=1;
    //---------------------------------------------------------------------------------

    ibuff = kmalloc(SIZE_BUF,GFP_KERNEL);
    if (ibuff == NULL) {
	printk(KERN_ALERT "%s: VM for reading buffer allocation failed\n",DevName);
	goto err_out;
    }
    ibuffw = kmalloc(SIZE_BUF,GFP_KERNEL);
    if (ibuffw == NULL) {
	printk(KERN_ALERT "%s: VM for writing buffer allocation failed\n",DevName);
	goto err_out;
    }

    //*******************************************************************************************************    

    for (sbrd=0; sbrd<max_card; sbrd++) {
	if (subcard[sbrd].BaseAdrRAM!=NULL) {
	    if (subboard[sbrd]) {// if subboard present.....
		reset_subcard(sbrd);//RESET (CS_RESET+...)
		read_rom_subcard(sbrd, NULL, 1);//ROM (CS_ROM+...)
	    }
	}
    }

    //------- инициализация таймера и проч.  --------------------------------------------------
    my_msec=0;
    timi=timi_def;
    bt=0;

    for (i=0; i<=kol_card; i++) Device_Open[i]=0;

    b_rtp=0; 
    subboard_flag=0;
    atomic_set(&dss_only, b_rtp);
    for (i=0; i<kol_card; i++) {

	spin_lock_init(&vin_lock[i]);
	spin_lock_init(&sre_lock[i]);
	spin_lock_init(&rd_lock[i]);
	spin_lock_init(&wr_lock[i]);

	reset_vinetic(i);

	ixc = (i<<9) + CS_VIN_ADR;
	if (subcard[i].type) {
	    ixa = (i<<9) + CS_VINETICe;
	    ixb = ixa + CS_STATUS_REGISTRe;
	    ixa <<= 4;
	    ixb <<= 4;
	    ixc <<= 4;
	} else {
	    ixa = (i<<9) + CS_VINETIC;
	    ixb = ixa + CS_STATUS_REGISTR;
	}
	VinAdrData[i] = (unsigned int)BasedByCard(i) + ixa;
	VinAdrStat[i] = (unsigned int)BasedByCard(i) + ixb;
	VinAdrAdr[i] = (unsigned int)BasedByCard(i) + ixc;

	memset(vrm_rd[i],0,len_vrm);
	memset(vrm_wr[i],0,len_vrm);
	atomic_set(&ops[i], b_rtp);
	atomic_set(&ips[i], b_rtp);
	atomic_set(&rd_error[i], b_rtp);
	atomic_set(&wr_error[i], b_rtp);

	wr_clr_all_packs(i);
	atomic_set(&ips_list[i], b_rtp);

	atomic_set(&bxsr[i], b_rtp);
	atomic_set(&hwsr[i], b_rtp);
	atomic_set(&begin_rtp[i], b_rtp);

	PrnSubCard(i);

	for (j=0; j<4; j++) {
	    k=(i<<2)+j;

	    RK_index[k] = ((k>>2)<<9) | ((k&3)<<6);
	    RK_len_buff[k]=0;
	    atomic_set(&sim900_len[k],0);
	    atomic_set(&sim900_txd[k],0);
	    atomic_set(&sim900_type[k],0);
	    atomic_set(&sim900_flag[k],1);

	    clr_all_packs(k);
	    atomic_set(&ops_list[k], b_rtp);

	    atomic_set(&rtp_on[k], b_rtp);
	    atomic_set(&rtp_init[k], b_rtp);
	    atomic_set(&rtp_init2[k], b_rtp);

	    atomic_set(&rk_rtp[i][j], b_rtp);
	    atomic_set(&rk2_rtp[i][j], b_rtp);
	    atomic_set(&set_mix_param_rtp[i][j], b_rtp);

	    atomic_set(&evt_rtp[i][j], 101);
	    atomic_set(&set_mix2_param_rtp[i][j], b_rtp);

	    atomic_set(&param_rtp[i][j], b_rtp);
	    atomic_set(&set_mix_param_rtp[i][j], b_rtp);
	    atomic_set(&near_echo_param[i][j], b_rtp);
	    atomic_set(&set_near_echo_param[i][j], b_rtp);
	    atomic_set(&far_echo_param[i][j], b_rtp);
	    atomic_set(&set_far_echo_param[i][j], b_rtp);
	    atomic_set(&done_rtp[i][j], b_rtp);

	    atomic_set(&gsm_flag[k], none);

	    atomic_set(&to_vin[k], b_rtp);
	    atomic_set(&from_vin[k], b_rtp);
	}
	cntr[i] = cntr_def;
	zero[i]=zero_def;//(500) - 1 sec 
    }

    atomic_set(&tick_mode,(int)timi_def);
    atomic_set(&varta, my_msec);
    init_timer(&my_timer);
    my_timer.function = MyTimer;
    my_timer.expires = jiffies + 100;	// 500 msec
    add_timer(&my_timer);
    my_timer_ready=1;

    memset(stri,0,all_radio+1); for (bt=0; bt<kol_card; bt++) sprintf(stri+strlen(stri),"%d",subboard[bt]);
    printk(KERN_ALERT "%s: subboards : %s\n", DevName, stri);

    memset(stri,0,all_radio+1); for (bt=0; bt<all_radio; bt++) sprintf(stri+strlen(stri),"%d",rk_present[bt]);
    printk(KERN_ALERT "%s: channels  : %s\n", DevName, stri);

    rc=0;
    goto dones;

err_out:

    if (ibuff!=NULL) kfree(ibuff);
    if (ibuffw!=NULL) kfree(ibuffw);

    for (k=0; k<max_brd; k++) {
	if (BaseAdr[k]!=NULL) iounmap(BaseAdr[k]);
	if (gsm_board[k]!=NULL) {
	    //pci_release_region(gsm_board[k],1);//BAR1
	    pci_release_regions(gsm_board[k]);
	    pci_disable_device(gsm_board[k]);
	}
    }


    if (my_dev_ready==1) {
	unregister_chrdev_region(MKDEV(Major, 0), 1);
	CLASS_DEV_DESTROY(altera_class, MKDEV(Major, 0));
	class_destroy(altera_class);

	unregister_chrdev_region(MKDEV(MajorV, 0), kol_card);
	deinit_sio(&chan_sio);
	for(k = 0; k <= kol_card; k++) CLASS_DEV_DESTROY(vin_class, MKDEV(MajorV, k));
	class_destroy(vin_class);
    }
    printk(KERN_ALERT "%s: Abort loading... Release all.\n",DevName);

    rc = -ENOMEM;

dones:

    return rc;

}
//************************************************************
//                 выгрузка модуля
//************************************************************
static void __exit rchan_exit(void)
{
int k;

    if (my_timer_ready==1) del_timer(&my_timer);

    if (ibuff!=NULL) kfree(ibuff);
    if (ibuffw!=NULL) kfree(ibuffw);

    for (k=0; k<max_brd; k++) {
	if (gsm_board[k]!=NULL) {
	    if (BaseAdr[k]!=NULL) iounmap(BaseAdr[k]);
	    //pci_release_region(gsm_board[k],1);//BAR1
	    pci_release_regions(gsm_board[k]);
	    pci_disable_device(gsm_board[k]);
	}
    }

    if (my_dev_ready==1) {
	unregister_chrdev_region(MKDEV(Major, 0), 1);
	CLASS_DEV_DESTROY(altera_class, MKDEV(Major, 0));
	class_destroy(altera_class);

	unregister_chrdev_region(MKDEV(MajorV, 0), kol_card);
	deinit_sio(&chan_sio);
	for(k = 0; k <= kol_card; k++) CLASS_DEV_DESTROY(vin_class, MKDEV(MajorV, k));
	class_destroy(vin_class);
    }
    printk(KERN_ALERT "%s: Release all. Remove driver.\n",DevName);

    return;

}

module_init(rchan_init);
module_exit(rchan_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("SalaraSoft <salara@ltd.com.ua>");
