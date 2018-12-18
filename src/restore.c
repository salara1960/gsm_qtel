#include "func.h"
// ************************************************************************
//   		M10/M12 GSM module writer		13.11.2015
// ************************************************************************

int main (int argc, char *argv[])
{

struct sigaction Act, OldAct;

unsigned char ind = 0, rx_faza = 0, tx_faza = 0, ef = 0, pack = 1, pck = 1, sync_ind = 0, sync_done = 0, tp = 0, pgm = 0;
char chaka[512] = {0};
char par_line[80] = {0};
char err_str[80] = {0};
unsigned int dlz, tmr_frame;
int lens = 0, i, wait_lens = 11, eoj = 0, stat_err = 0, seq_err = 0;
char *ptr = NULL;
char *pointerRX = NULL;
time_t start_t, stop_t;

#ifdef TEST_MODE
unsigned char ack_begin[11] = {0xaa,0x00,0x02,0x00,0x04,0x00,0x00,0x04,0x00,0x25,0x21};
unsigned char ack_end[9] = {0xaa,0x00,0x06,0x00,0x02,0x00,0x00,0xa3,0xe5};
unsigned char ack_run[9] = {0xaa,0x00,0x08,0x00,0x02,0x00,0x03,0x5c,0x2e};
#else
unsigned char byte;
int rt;
#endif


    if (argc < 3) {
	printf("\nYou must enter channel_number and file_name.\n\
	       for example: writer 1 firmware.bin\n\twhere 1 - channel number (1..24/32), firmware.bin - firmware binary file\n");
	return -1;
    } else {
	memset(par_line, 0, sizeof(par_line));
	strcpy(par_line, argv[1]);
	dlz = atoi(argv[1]);
	dlz--;
	if ((dlz < all_radio) && (dlz >= 0)) Def_RK = dlz;
	else {
	    printf("\nInvalid channel number %d\n", dlz + 1);
	    return -1;
	 }
	memset(fname, 0, sizeof(fname));
	strcpy(fname, argv[2]);
	if (argc > 3) {
	    memset(chaka, 0, sizeof(chaka));
	    strcpy(chaka, argv[3]);
	    if (strstr(chaka,"dump") != NULL) dmp = 1;
	}
    }

    start_t = stop_t = time(NULL);

//------------  set signal route functions ------------------

    memset((unsigned char *)&Act, 0, sizeof(struct sigaction));
    memset((unsigned char *)&OldAct, 0, sizeof(struct sigaction));
    Act.sa_handler = &_SigProc;
    Act.sa_flags   = 0;
    sigaction(SIGHUP,  &Act, &OldAct);
    sigaction(SIGTERM, &Act, &OldAct);
    sigaction(SIGINT,  &Act, &OldAct);
    sigaction(SIGSEGV, &Act, &OldAct);

    printf("\n%s. Start writer for channel %d with file '%s' (PID %d)\n", CurTime(), Def_RK + 1, fname, getpid());

//------------------------------ читаем bin-файл ----------------------------------------

    if (ReadBIN(fname) == -1) {
	printf("%s. Can't read file %s\n",CurTime(),fname);
	if (bin) free(bin);
        return -1;
    } else printf("%s. File %s readed Ok (size: file=%d buf=%d, pk_all=%d, last_pk_len=%d)\n",
		    CurTime(),fname,file_size,i,pk_all,last_size);

//---------------- открываем device радиоканалов ----------------------------------------

    if ((fd = open("/dev/k16", O_RDWR) ) < 0) {
	printf("Can't open /dev/k16\n");
	if (bin) free(bin);
        return -1;
    }

//----------------------------  open TMR device ----------------------------------------------

    if ((fd_timer = open("/dev/tmr", O_RDWR) ) < 0) {
	printf("Can't open /dev/tmr (%s)\n", strerror(errno));
	if (bin) free(bin);
        return -1;
    }

//---------------------------- startup initialize -------------------------------------------

    reset_timer_tmr(255);//сбросить в нуль оба таймера (10ms & 1ms)

    ind_rk(Def_RK);//create offset adress and mirror for order channel

    SetTickMode(1);//timer to 1 msec (for g20)  OR  Set DSS_ONLY (for i32)

    SayToCore(Def_RK);//present_rk to kernel driver (k16_mod)

    SetKT(Def_RK);

//--------------  init COM.xx  и прочего хлама   ---------------------------------------------

    memset((unsigned char *)&COM.StatusAdp, 0, sizeof(s_COM));

    tmr0 = get_timer(_30s);
    tmr_ms = get_timer(_50ms);
    tmr_frame = get_timer(_10s);
    tmr_rx = get_timer(_1s);
    tmr_one = get_timer_tmr(0);
    loops = 1;
    faza = last_faza = 0;
    vio = last_vio = 0;
    at_data = 0;

    pointerRX = (char *)&COM.RX_com[0];

    ModulePWR_OFF(Def_RK);

//**************************************************************************************************
//				MAIN LOOP
//**************************************************************************************************

    while (loops) {

	if (!at_data) {
	    if (check_timer_tmr(tmr_one)) {
		check_vio(Def_RK);
		tmr_one = get_timer_tmr(2);
	    }
	}

	//-------------------------     AT     ------------------------------------
	faza = COM.faza;
	switch (faza) {
	    case 0 ://MODULE_ON
		if (check_delay(tmr_ms)) {
		    ind = 0;//uk to first cammand
		    if (vio) {
			COM.Count_Error = 0;
			tmr_ms = get_timer(_1s);
			tmr0 = get_timer(_30s);
			faza = 5;
		    } else {
			Module_ON(Def_RK);
			tmr_ms = get_timer(_20ms);
			faza = 1;
		    }
		}
	    break;
	    case 1://PWR_ON
		if (check_delay(tmr_ms)) {
		    ModulePWR_ON(Def_RK);
		    tmr_ms = get_timer(_2s);
		    faza = 2;
		}
	    break;
	    case 2://MODULE_OFF
		if (check_delay(tmr_ms)) {
		    Module_OFF(Def_RK);
		    tmr_ms = get_timer(_2_5s);
		    faza = 3;
		}
	    break;
	    case 3://CHECK_VIO
		if (vio) {
		    COM.Count_Error = 0;
		    tmr_ms = get_timer(_5s);
		    faza = 4;
		}
		if (check_delay(tmr_ms)) {
		    tmr_ms = get_timer(_5s);
		    faza = 4;
		}
	    break;
	    case 4://WAIT MSG
		if (COM.StatusAdp & ReciveCom) {
		    COM.StatusAdp &= ReciveCom ^ 0xFF;//сбросить признак приема ответа
		    if (strstr(pointerRX,"+CPIN:")) {
			if (!on_off) Module_ON(Def_RK);
			COM.Count_Error = 0;
			tmr_ms = get_timer(_1s);
			faza = 5;
		    }
		} else if (check_delay(tmr_ms)) {
		    if (!vio) {
			ModulePWR_OFF(Def_RK);
			tmr_ms = get_timer(_5s);
			faza = 0;
		    } else {
			COM.Count_Error = 0;
			ind = 255;
			if (ChangeSpeed(Def_RK)) put_AT_com("AT+IPR=9600");//old_speed=115200, new_speed=9600
					    else put_AT_com("AT+IPR=115200");//old_speed=9600, new_speed=115200
			tmr_ms = get_timer(_5s);
			faza = 6;
		    }
		}
	    break;
	    case 5://AT
		if (check_delay(tmr_ms)) {
		    memset(txd,0,max_buf_len);
		    strcpy(txd, &cmd[ind][0]);
		    put_AT_com(txd);
		    tmr_ms = get_timer(_5s);
		    faza = 6;
		}
	    break;
	    case 6://wait OK
		if (COM.StatusAdp & ReciveOK) {
		    COM.StatusAdp &= ReciveOK^0xFF;//сбросить признак приема OK
		    COM.Count_Error = 0;
		    tmr_ms = get_timer(_20ms);
		    faza = 5;
		    ind++;
		    if (ind >= max_at) { ind = 0; faza = 254; }
		} else if (check_delay(tmr_ms)) {
		    COM.Count_Error++;
		    tmr_ms = get_timer(_500ms);
		    faza = 5;
		    if (COM.Count_Error >= max_err) {
			COM.Count_Error = 0;
			ind++;
			if (ind >= max_at) { ind = 0; faza = 254; }
		    }
		} else if (COM.StatusAdp & ReciveERR) {//выполнять, если получен ERROR
		    COM.StatusAdp &= ReciveERR ^ 0xFF;
		    COM.Count_Error++;
		    tmr_ms = get_timer(_1s);
		    faza = 5;
		    if (COM.Count_Error >= max_err) {
			COM.Count_Error = 0;
			ind++; if (ind >= max_at) { ind = 0; faza = 254; }
		    }
		} else if (COM.StatusAdp & ReciveCME) {
		    COM.StatusAdp &= ReciveCME ^ 0xFF;//сбросить признак приема ответа
		    COM.Count_Error++;
		    tmr_ms = get_timer(_1s);
		    faza = 5;
		    if (COM.Count_Error >= max_err) {
			COM.Count_Error = 0;
			ind++; if (ind >= max_at) { ind = 0; faza = 254; }
		    }
		}
	    break;
	    //-----------------------------------------------------------------------
	    case 10://ALL CMD
		if (check_delay(tmr_ms)) {
		    memset(rxd, 0, max_buf_len);
		    uk_rxd = &rxd[0];
		    lens = 0;
		    switch (pack) {
			case 1://CMD_DL_BEGIN
			    mk_dl_begin(dmp);
			    wait_lens = 11;
#ifdef TEST_MODE
			    lens = sizeof(ack_begin);
			    memcpy(uk_rxd, &ack_begin[0], lens);
#endif
			break;
			case 2://CMD_DL_DATA
			    mk_dl_data(dmp, pk_len, seq_number);
			    wait_lens = 13;
#ifdef TEST_MODE
			    lens = mk_ack_data(seq_number+1, (unsigned char *)uk_rxd);
#endif
			break;
			case 3://CMD_DL_END
			    mk_dl_end(dmp);
			    wait_lens = 9;
#ifdef TEST_MODE
			    lens = sizeof(ack_end);
			    memcpy(uk_rxd, &ack_end[0], lens);
#endif
			break;
			case 4://CMD_RUN_GSMSW
			    mk_run_gsmsw(dmp);
			    wait_lens = 9;
#ifdef TEST_MODE
			    lens = sizeof(ack_run);
			    memcpy(uk_rxd, &ack_run[0], lens);
#endif
			break;
			    default : eoj=1;
		    }
		    if (eoj) faza = 251;
		    else {
			tmr_ms = get_timer(_5s);
#ifdef x86
			usleep(1);
#else
			usleep(100);
#endif
			faza = 11;
		    }
		}
	    break;
	    case 11:
#ifndef TEST_MODE
		if ((RXdone(Def_RK)) && (!eoj)) {
		    rt = RXbyte(Def_RK, 0);
		    if (rt != -1) {
			byte = rt;
			*uk_rxd = byte; uk_rxd++; lens++;
#endif
			if (lens == wait_lens) {
			    pk_status = parse_answer(dmp, (unsigned char *)&rxd[0], lens, &seq_number_resp, &pk_max_dl);
			    if ((pk_status >= 0) && (pk_status < max_status)) i = pk_status; else i = max_status - 1;
			    sprintf(chaka,"%s ANSWER(%d): cmd: pack=%d rx=%d, status=%d(%s)", DTNowPrn4(labka), lens, pack, hdr_resp.cmd, pk_status, (char *)msg_status[i]);
			    if (pk_status) stat_err++;
			    if (stat_err) sprintf(chaka+strlen(chaka),", stat_err=%d",stat_err);
			    pck = pack;
			    switch (pack) {
				case 1://CMD_DL_BEGIN
				    if (!pk_status) {
					sprintf(chaka+strlen(chaka)," pack_max_len=%d",pk_max_dl);
					pck = 2;//goto CMD_DL_DATA
				    } else eoj = 1;//pck = 3;//goto CMD_DL_END
				break;
				case 2://CMD_DL_DATA
				    sprintf(chaka+strlen(chaka)," seq_numbers: tx=%d rx=%d",seq_number,seq_number_resp);
				    if ((seq_number + 1) != seq_number_resp) seq_err++;
				    if (seq_err) sprintf(chaka+strlen(chaka)," seq_err=%d",seq_err);
				    seq_number++;
				    if (!pk_status) {
					if (seq_number < pk_all) {
					    if (seq_number == (pk_all-1)) pk_len = last_size;
					    pck = 2;//goto CMD_DL_DATA
					} else pck = 3;//goto CMD_DL_END
				    } else pck = 3;//goto CMD_DL_END
				break;
				case 3: pck = 4; break;//CMD_DL_END
				case 4: eoj = 1;   break;//CMD_RUN_GSMSW
				    default : eoj = 1;//goto end of job
			    }
			    pack = pck;
			    if ((dmp) || (stat_err + seq_err)) {
			        sprintf(chaka+strlen(chaka),"\n");
			        printf(chaka);
			    }
			    if (!eoj) {
				tmr_ms = get_timer(0);
				faza = 10;
			    } else {
				stop_t = time(NULL);
				faza = 250;//251
			    }
			    break;
			}
#ifndef TEST_MODE
		    }
		}
#endif
	        if ((check_delay(tmr_ms)) || (eoj)) faza = 250;//251
	    break;
	    //-----------------------------------------------------------------------
	    case 20:
		if (check_delay(tmr_ms)) {
#ifndef TEST_MODE
		    if (COM.Count_Error == 3) ModulePWR_ON(Def_RK);
#endif
		    if (TXdone(Def_RK)) {
			TXbyte(Def_RK, SYNC_WORD[sync_ind], dmp);
			tmr_ms = get_timer(_20ms);
			COM.Count_Error++;
			faza = 21;
		    }
		}
	    break;
	    case 21:
		if (RXdone(Def_RK)) {
		    if (RXbyte_prn(Def_RK, SYNC_WORD_RESP[sync_ind], dmp) == SYNC_WORD_RESP[sync_ind]) {
			sync_ind++;
			if (sync_ind == 2) { sync_done = 1; tmr_frame = get_timer(0); }
		    }
		}
		if (check_delay(tmr_frame)) {
#ifdef TEST_MODE
		    sync_done = 1;
#endif
		    if (!sync_done) {
			tmr_ms = get_timer(_20ms);
			faza = 250;//251
		    }
		    else {
			at_data = 1;
			seq_number = 0;
			pk_len = max_data_len;
			pack = pck = 1;
			ClearRX();//clear rx_buf
			printf("%s Start download process.\n",DTNowPrn4(labka));
			start_t = time(NULL);
			tmr_ms = get_timer(_20ms);
			faza = 10;
		    }
		    break;
		} else {
		    if (check_delay(tmr_ms)) faza = 20; else faza = 21;
		}
	    break;
	    case 251:
		tmr_ms = get_timer(_1s);
		loops = 0;
	    break;
	    case 250:
		if (check_delay(tmr_ms)) {
		    if (!pgm) {
			tmr_frame = get_timer(_5s);//_300ms
			tmr_ms = get_timer(0);
			pgm = 1;
			faza = 20;
		    } else {
			at_data = 0;
			rx_faza = 0;
			tx_faza = 0;
			tmr_ms = get_timer(_2s);
			faza = last_faza = 2;//goto MODULE_OFF
			if (!pwr_on_off) ModulePWR_ON(Def_RK);
		    }
		}
	    break;
	    //-----------------------------------------------------------------------
	    case 252:	//unsigned char SYNC_WORD[2] = {0xB5, 0xA9};
			//unsigned char SYNC_WORD_RESP[2] = {0x5B, 0x9A};
		if (pwr_on_off) ModulePWR_OFF(Def_RK);
		tp = 0;
		if (strstr(module,"M10ER01A")) tp = 10;
		else if (strstr(module,"M12ER01A")) tp = 12;
		if (tp) {
		    tmr_ms = get_timer(_1_5s);
		    COM.Count_Error = 0;
		    sync_ind = 0;
		    sync_done = 0;
		    rx_faza = 0;
		    tx_faza = 0;
		    if (!pgm) {
			ClearRX();
			printf("%s Start sync process.\n",DTNowPrn4(labka));
			at_data = 1;//GO TO DATA MODE !!!!!!!!!!!!!!!!!!!!
			faza = 250;//20;
		    } else {
			at_data = 0;
			faza = 251;
		    }
		} else faza = 251;
	    break;
	    case 253:
		if ( (!vio) || (check_delay(tmr_ms)) ) {
		    ModulePWR_OFF(Def_RK);
		    faza = 252;
		}
	    break;
	    case 254 ://goto out
		if (check_delay(tmr_ms)) {
		    if (!vio) {
			tmr_ms = get_timer(_1s);
			loops = 0;
			break;
		    }
		    if (on_off) Module_OFF(Def_RK);
		    tmr_ms = get_timer(_5s);
		    faza = 253;
		}
	    break;
	    //-----------------------------------------------------------------------
	    case 255 :
		printf("ERROR faza %d !!!\n", ef);
		faza = 254;
	    break;
		default : { ef = faza ; faza = 255; }
	}
	COM.faza = faza;
/*
	if (last_faza!=faza) {
	    last_faza=faza;
	    print_faza(faza);
	}
*/
	//------------------------     RX    -----------------------------------------------------
	rx_faza = COM.rx_faza;
	switch (rx_faza) {
	    case 0:
		if (!at_data) {//AT
		    if (RXdone(Def_RK)) {
			memset(COM.RX_com, 0, max_buf_len);
			COM.rx_numb = 0;
			COM.rx_ready = 0;
			tmr_rx = get_timer(_1_5s);
			rx_faza = 1;
		    }
		} else rx_faza = 4;//DATA MODE
	    break;
	    case 1:
		if ((RXdone(Def_RK)) && (COM.rx_numb<max_buf_len) && (!COM.rx_ready)) {
		    memset(rxd, 0, max_buf_len);
		    lens = RXbytes22(Def_RK, rxd, at_data);
		    if (lens > 1) {
			ptr = (char *)&COM.RX_com[COM.rx_numb];
			if ((COM.rx_numb + lens) > max_buf_len) {
			    lens = (unsigned short)max_buf_len - COM.rx_numb - 1;
			    if (lens > 0) rxd[lens - 1] = 0x0d;
			} else COM.rx_numb = COM.rx_numb + lens;
			memcpy(ptr, rxd, lens);
			if (COM.RX_com[COM.rx_numb-1] == 0x0d) COM.rx_ready = 1;
		    }
		}
		if (COM.rx_ready == 1) rx_faza = 2;
		else {
		    if (check_delay(tmr_rx)) { if (!COM.rx_numb) rx_faza = 0; else rx_faza = 2; }
		}
	    break;
	    case 2:
		if (strstr(pointerRX,"OK")) COM.StatusAdp |= ReciveOK;
		else
		if (strstr(pointerRX,"ERROR")) COM.StatusAdp |= ReciveERR;
		else
		if (strstr(pointerRX,"+CME")) COM.StatusAdp |= ReciveCME;
		else {
		    COM.StatusAdp |= ReciveCom;// какая-то другая команда (точнее ответ на команду)
		    if (strstr(pointerRX,"Revision: ")) {
			if (!pgm) {
			    memset(module, 0, max_buf_len);
			    strcpy(module, pointerRX + 10);
			    i = strlen(module);
			    if (i > 0) {
				if (module[i - 1] == 0x0d) module[i - 1] = 0;
			    }
			} else {
			    memset(module_new, 0, max_buf_len);
			    strcpy(module_new, pointerRX + 10);
			    i = strlen(module_new);
			    if (i > 0) {
				if (module_new[i - 1] == 0x0d) module_new[i - 1] = 0;
			    }
			}
		    }
		}
		memset(rxd, 0, max_buf_len);
		sprintf(rxd, "rx: %s", pointerRX);
		string_AT_com(rxd);
		rx_faza = 0;
	    break;
	    case 4:
		rx_faza = 0;
	    break;
	}
	COM.rx_faza = rx_faza;
	//------------------------     TX    -----------------------------------------------------
	tx_faza = COM.tx_faza;
	switch (tx_faza) {
	    case 0: break;
	    case 1:
		COM.StatusAdp = 0; //статус СОМ-порта радиоканала в исх. состояние
		COM.MainError = 0; //очистить код ошибки СОМ-порта радиоканала
		tx_faza=2;
	    break;
	    case 2:
		tx_faza = TXCOM2_all(Def_RK, at_data);
	    break;
	}
	COM.tx_faza = tx_faza;
#ifndef x86
	if (!at_data) usleep(1);
#else
	if (!at_data) usleep(100);
#endif

    }


    if (pwr_on_off) ModulePWR_OFF(Def_RK);

    SetTickMode(0);//timer to 2 msec (for g20) OR Unset DSS_ONLY (for i32)

//**************************************************************************************************

    eoj = (int)stop_t - (int)start_t;

    printf("%s. time=%d sec., chan=%d, err=%d/%d, packs=%d, revision: %s | %s\n",
		CurTime(), eoj, Def_RK+1, stat_err, seq_err, seq_number, module, module_new);

    my_CloseAll();

    return 0;

}



