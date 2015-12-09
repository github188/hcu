/*
 * apigprs.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "apigprs.h"
#include "../comvm/trace.h"

/*
** Local static variables
*/

//static unsigned char traceFileName[] = __FILE__;
//static char gGprsConfigFile[] = "/etc/ppp/peers/gprs";

//static char BufSerial[HCU__MAX_LENGTH_CMD_SERIAL];

/*
 * Initialize the GPRS parameters from local configuration
 */
UINT32 GprsInit(SerialPort_t *pGprsSerialPortConfig, GprsPppdConf_t *gpc)
{

	UINT8 id = pGprsSerialPortConfig->id;				/* COM1=>0, COM1=>1, COM2=>2 ....  */
	//UINT32 nSpeed = pGprsSerialPortConfig->nSpeed;		/* 1200, 2400, 4800, 4800, 9600, 19200, 38400, 57600, 115200 */
	//UINT16 nBits = pGprsSerialPortConfig->nBits;		/* 7 or 8 */
	//UINT8 nEvent = pGprsSerialPortConfig->nEvent;		/* '0', 'N', 'E' */
	//UINT16 nStop = pGprsSerialPortConfig->nStop;		/* 0, or 1 */
	UINT16 fd = pGprsSerialPortConfig->fd;

	FILE *fp;
	UINT32 cnt = 0;
	UINT32 ret;
	//UINT32 len = 0;
	//UINT32 readlen = 0;
	int NumOfLine = 0;
	char buf[1024];
	char out[8][1024];

	char CurTime[18]; /* +CCLK: "13/06/01,23:43:06" -> 13/06/01,23:43:06 */
	/*  start from 8:    01234567890123456789012345                      */
	/*  len is 17:                                     12345678901234567 */
	CurTime[17] = 0x00;

	HcuDebugPrint("GprsInit: GPRS and PPPD not actived, checking GPRS module ...\n");

	ret = SerialPortOpen(id, &fd);
	if (FAILURE == ret)
	{
		HcuErrorPrint("GprsInit: serial port COM%d open failure.\n", id);
		return FAILURE;
	}
	HcuDebugPrint("GprsInit: serial port COM%d open successfully, fd saved as %d.\n", id, fd);

	/* Save the handler back to Global variables */
	pGprsSerialPortConfig->fd = fd;

	ret = SerialPortSet(pGprsSerialPortConfig);
	if (FAILURE == ret)
	{
		HcuErrorPrint("GprsInit: Open Serial Port %d failure.\n", id);
		return ret;
	}
	HcuDebugPrint("GprsInit: InitSerialPort serial port COM%d properties has been set successfully.\n", id);

	/* Chang the serial from 0, 1 to 2, 1, enable serial time out*/
	SerialPortSetVtimeVmin(pGprsSerialPortConfig, 2, 0);

	//bzero(BufSerial, HCU_MAX_LENGTH_CMD_SERIAL);
	//readlen = read(fd, BufSerial, HCU_MAX_LENGTH_CMD_SERIAL);

	/* Send first AT Command */
	/* Auto band rate is not implemented, use AT Command before Run HCU, Set: AT+IPR=<rate>, Get: AT+IPR? */
	ret = AtCommand(pGprsSerialPortConfig, "AT\r", *out, &NumOfLine);
	if (ret == FAILURE)
	{
		HcuErrorPrint("GprsInit: GPRS Modem is not ready, make sure BandRate is correct, via AT+IPR=<rate> firstly, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("GprsInit: GPRS Modem Ready.\n");
	}
	else
	{
		HcuErrorPrint("GprsInit: GPRS Modem is not ready, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	/* Send first Check the GPRS Model Version */
	ret = AtCommand(pGprsSerialPortConfig, "AT+CGMR\r", *out, &NumOfLine);
	if (ret == FAILURE)
	{
		HcuErrorPrint("GprsInit: GPRS Modem is not ready, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("GprsInit: GPRS Modem HW/SW verion read ok.\n");
	}
	else
	{
		HcuErrorPrint("GprsInit: GPRS Modem is not ready, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	ret = AtCommand(pGprsSerialPortConfig, "AT+CPIN?\r", *out, &NumOfLine);
	if (ret == FAILURE)
	{
		HcuErrorPrint("GprsInit: GPRS SIM CARD is not ready, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("GprsInit: GPRS SIM CARD OK\n");
	}
	else
	{
		HcuErrorPrint("GprsInit: GPRS SIM CARD is not ready, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}


	ret = AtCommand(pGprsSerialPortConfig, "AT+CREG?\r", *out, &NumOfLine);

	if (ret == FAILURE)
	{
		HcuErrorPrint("GprsInit: GPRS SIM CARD is not REGISTRED, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("GprsInit: GPRS SIM CARD REGISTERED TO NETWORK\n");
	}
	else
	{
		HcuErrorPrint("GprsInit: GPRS SIM CARD is not REGISTRED, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	/* Get GPRS system time */
	ret = AtCommand(pGprsSerialPortConfig, "AT+CCLK?\r", *out, &NumOfLine);

	if (ret == FAILURE)
	{
		HcuErrorPrint("GprsInit: GPRS READ Realtime Clock failure, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		/* Get the time string from GPRS module */
		strncpy(CurTime, out[NumOfLine-3]+8, 17);
		HcuDebugPrint("GprsInit: GPRS READ Realtime Clock OK: %s\n", CurTime);
		if(HCU_LOGIC_TRUE == gpc->SyncSysTimeFromGprs)
		{
			SetSystemTime(CurTime);
		}
	}
	else
	{
		HcuErrorPrint("GprsInit: GPRS READ Realtime Clock failure, Serial Port COM%d closed ...\n", id);
		SerialPortClose(fd);
		return FAILURE;
	}

	/* return successfully */
	SerialPortClose(fd);
	HcuDebugPrint("GprsInit: GPRS status checking OK.\n");

	/* PPP Configuration File */
	/* "/etc/ppp/peers/gprs" */

	fp = fopen(gpc->PppConfigFileName, "r");
	if (NULL == fp)
	{
		HcuErrorPrint("GprsInit: GPRS scripts [%s] open error ...\n", gpc->PppConfigFileName);
		return FAILURE;
	}

	while(!feof(fp))
	{
		fgets(buf, 1023, fp);
		HcuDebugPrint("GprsInit: GPRS scripts [%s] line #%02d: %s", gpc->PppConfigFileName, cnt, buf);
		cnt++;
	}

	HcuDebugPrint("GprsInit: GPRS scripts [%s] not changed for this version of the HCU ...\n", gpc->PppConfigFileName);

	/* PPP Dail Command */
	/* "/usr/bin/pppd" */
	/* Check local HCU configuration table for GPRS port */
	/* Apply these configuration in the OS's scripts */

	return SUCCESS;

}


UINT32 GprsPppdStart(GprsPppdConf_t *gpc)
{

	/* "/usr/bin/pppd" */
	int ret = 0;
	int i;

	system("ifconfig eth0 down");
	HcuDebugPrint("GprsPppStart: Bring down eth0 via [ifconfig eth0 down] ...\n", ret);

	ret = system("/usr/bin/pppd call gprs&");
	if (-1 == ret)
	{
		HcuErrorPrint("GprsPppStart: pppd command not found, can not start GPRS ...\n");

		system("ifconfig eth0 up");
		HcuDebugPrint("GprsPppStart: Bring up eth0 via [ifconfig eth0 up] ...\n", ret);

		return FAILURE;
	}
	else if (0 != ret)
	{
		HcuErrorPrint("GprsPppStart: pppd un-successful, ret = %d ...\n", ret);

		system("ifconfig eth0 up");
		HcuDebugPrint("GprsPppStart: Bring up eth0 via [ifconfig eth0 down] ...\n", ret);

		return FAILURE;
	}

	/* usleep for 20 sec */
	for (i = 0; i < (gpc->GprsPppdWaitSec); i++)
	{
		usleep(NB_MICROS_IN_ONE_SECOND);
		HcuDebugPrint("GprsPppStart: wait for pppd call gprs& to complete, delay %02d sec ...\n", i+1);
	}

	system("ifconfig eth0 up");
	HcuDebugPrint("GprsPppStart: Bring up eth0 via [ifconfig eth0 up] ...\n", ret);
	usleep(NB_MICROS_IN_ONE_SECOND);

	/* Check the GPRS and PPPD status */
	ret = IsGprsPppdOk();
	if (FAILURE == ret)
	{
		HcuDebugPrint("GprsInit: GPRS and PPPD actived failure ...\n");
		return FAILURE; /* There is already GPRS actived */
	}

	HcuDebugPrint("GprsPppStart: pppd successful, ret = %d.\n", ret);
	HcuDebugPrint("GprsPppStart: refresh system information ...\n", ret);
	ret = HcuGetSysInfo(&gSysInfo);

	return SUCCESS;

}

UINT32 GprsPppStop()
{
	return 0;
}

UINT32 GprsPingSever()
{
	return 0;
}

void GprsHwReset(GprsPppdConf_t *gpc)
{

	HcuDebugPrint("GprsHwReset: reset GPRS model via HW GPIO, start ...\n");

	/* Reset GPRS via GPIO */
	HcuDebugPrint("GprsHwReset: GPIO PortOp %s.\n", gpc->HwResetPortOp1);
	if (NULL != gpc->HwResetPortOp1)
	{
		if(strcmp(gpc->HwResetPortOp1, "NULL"))
		{
			GpioPortOp(gpc->HwResetPortOp1);
			GpioPortOp("m0=1");
		}
	}

	HcuDebugPrint("GprsHwReset: delay %d msec ...\n", gpc->HwResetPortOp1DelayMsec);
	usleep(NB_MICROS_IN_ONE_MS * gpc->HwResetPortOp1DelayMsec);

	HcuDebugPrint("GprsHwReset: GPIO PortOp %s.\n", gpc->HwResetPortOp2);
	if (NULL != gpc->HwResetPortOp2)
	{
		if(strcmp(gpc->HwResetPortOp2, "NULL"))
		{
				GpioPortOp(gpc->HwResetPortOp2);
				GpioPortOp("m0=0");
		}
	}

	HcuDebugPrint("GprsHwReset: delay %d msec ...\n", gpc->HwResetPortOp2DelayMsec);
	usleep(NB_MICROS_IN_ONE_MS * gpc->HwResetPortOp2DelayMsec);
	GpioPortOp("m0=1");

	HcuDebugPrint("GprsHwReset: reset GPRS model via HW GPIO, complete.\n");

}

UINT32 GprsPppdSystemStartup(GprsPppdConf_t *gpc)
{

	UINT32 ret;
	UINT32 TimesGprsCheck = 0;
	UINT32 TimesGprsReset = 0;

	/* First Check whether there is already GPRS and PPP setup */
	ret = IsGprsPppdOk();
	if (SUCCESS == ret)
	{
		HcuDebugPrint("GprsPppdSystemStartup: GPRS & PPP already start, nothing done.\n");
		return SUCCESS;
	}
	else /* if the Gprs is not working, try to shut down both of the network interface and pppd pprocess */
	{
		HcuDebugPrint("GprsPppdSystemStartup: GPRS & PPP not started, reset GPRS HW and shutdown PPPd and ppp0 interface.\n");
		GprsHwReset(gpc); /* Reset GPRS Module by PGIO */
		GprsPppdSwShutdown();  /* Clean the SW status */
	}
	/* after this, the GPRS model and Network interface and PPP process should be cleaned */

	/* If GPRS/PPP not active, prepare to strat, wait for GPRS model to startup */
	/* Initial GPRS */
	while(1) // STRATEGY TO BE DEFINED !!!!!!!!!
	{
		ret = GprsInit(&gSerialPortGprs, &gGprsPppdConf);
		if (SUCCESS == ret)
		{
			HcuDebugPrint("GprsPppdSystemStartup: GPRS module ok, launch ppd ...\n");
			break;
		}
		else
		{
			/* Increase the Check counter */
			TimesGprsCheck++;
			HcuErrorPrint("GprsPppdSystemStartup: GPRS module nok, checked %d times, will recheck in 10 seconds ...\n", TimesGprsCheck);
			if(TimesGprsCheck > gpc->MaxGprsCheckTimes)
			{

				TimesGprsReset++;

				HcuErrorPrint("GprsPppdSystemStartup: GPRS module nok total for %d times, max time reach, reset GPRS HW for %d times...\n", TimesGprsCheck, TimesGprsReset);
				/* HW reset if many times of checking failed !!!!!!!! */
				if (TimesGprsReset > gpc->MaxGprsResetTimes)
				{
					HcuErrorPrint("GprsPppdSystemStartup: GPRS module reset for %d times, max time reach, STOP ...\n", TimesGprsReset);
					return FAILURE;
				}

				TimesGprsCheck = 0;

				/* reset the GPRS module */
				GprsHwReset(gpc);
			}
		}

		usleep(NB_MICROS_IN_ONE_SECOND * 10);
	}

	/* decide whether we need to start GPRS */
	TimesGprsCheck = 0;
	/* Start PPPD over GPRS */
	while(1) // STRATEGY TO BE DEFINED !!!!!!!!!
	{
		ret = GprsPppdStart(gpc);
		if (SUCCESS == ret)
		{
			HcuDebugPrint("GprsPppdSystemStartup: Init GPRS PPP Success ...\n");
			break;
		}
		else
		{
			TimesGprsCheck++;
			if(TimesGprsCheck > gpc->MaxGprsCheckTimes)
			{
				HcuErrorPrint("GprsPppdSystemStartup: Init GPRS PPP failure %d times, Max time reach, STOP ...\n", TimesGprsCheck);
				return FAILURE;
			}
			HcuErrorPrint("GprsPppdSystemStartup: Init GPRS PPP failure %d times, will kill pppd process, and retry in 10 seconds ...\n", TimesGprsCheck);
			/* Kill all SW related PPP process and also the ppp0 interface */
			GprsPppdSwShutdown();
			//return ret;
		}

		usleep(NB_MICROS_IN_ONE_SECOND * 10);
	}

	/* Check whether it is OK */
	ret = IsGprsPppdOk();
	if (SUCCESS == ret)
	{
		HcuDebugPrint("GprsPppdSystemStartup: GPRS & PPP start successfully.\n");
		return SUCCESS;
	}

	/* NORMALLY SHOULD NOT STOP AT HERE */
	HcuDebugPrint("GprsPppdSystemStartup: GPRS & PPP STATUS NOK EVEN AFTER GPRS & PPP START, NEED TO CHECK, STOP.\n");
	return FAILURE;

}

void GprsPppdSwShutdown()
{
	UINT32 ret;

	/* Shutdown network interface */
	ret = IsNetItfExist("ppp0");
	if (SUCCESS == ret)
	{
		system("ifconfig ppp0 down");
		HcuDebugPrint("GprsPppdShutdown: Bring down ppp0 via [ifconfig ppp0 down]\n");
	}

	/* Kill PPPd Process if there is any, so that the serial port could be freed */
	KillPppdProcess();
	HcuDebugPrint("GprsPppdShutdown: pppd process cleaned to free serial port for GPRS model.\n");

	/* refred system information for network interface checking */
	HcuGetSysInfo(&gSysInfo);
	HcuDebugPrint("GprsPppdShutdown: refresh system information complete.\n");

}
/*
 * Send one AT Command, and wait for reply from the serails port
 *
 * In  : pGprsSerialPortConfig: Serial Port information, include fdl
 * In  : AtCmd: AT Command send to Modem
 * Out : ReplyStr: Each of the line of the AT command is save in ReplyStr[0], ReplyStr[1*1024], ReplyStr[(ReplayCnt-1)*1024]
 * Out : ReplyCnt: number of line expected from Modem (>=1)
 *
 */

UINT32 AtCommand(SerialPort_t *pGprsSerialPortConfig, char *AtCmd, char *ReplyStr, int *ReplyCnt)
{

	UINT32 cnt = 0;
	UINT32 i = 0;
	UINT32 len = 0;
	UINT32 readlen = 0;
	int NumOfLine = 0;
	UINT32 IdxOfCharInOneLine = 0;
	UINT16 fd = pGprsSerialPortConfig->fd;
	char buf[1024];

	/* Check input */
	if(NULL == pGprsSerialPortConfig)
	{
		HcuErrorPrint("AtCommand: pGprsSerialPortConfig = NULL, return.\n");
		return FAILURE;
	}

	if(HCU_INVALID_U16 == pGprsSerialPortConfig->fd)
	{
		HcuErrorPrint("AtCommand: pGprsSerialPortConfig->fd = 0xFFFF, serial port COM%d not openned, return.\n", pGprsSerialPortConfig->id);
		return FAILURE;
	}

	if(NULL == AtCmd)
	{
		HcuErrorPrint("AtCommand: Input pointer AtCmd = NULL, return.\n");
		return FAILURE;
	}

	if(NULL == ReplyStr)
	{
		HcuErrorPrint("AtCommand: Output pointer ReplyStr = NULL, return.\n");
		return FAILURE;
	}

	/* Send second AT Command */
	bzero(buf, 1024);
	len = write(fd, AtCmd, strlen(AtCmd));
	HcuDebugPrint("AtCommand: AT SEND: %s\n", AtCmd);

	usleep(NB_MICROS_IN_ONE_SECOND * 2);
	HcuDebugPrint("AtCommand: Delayed 2 sec to get response ...\n");

	/* Read All Reply */
	readlen = read(fd, buf, 1023);
	if (-1 == readlen)
	{
		HcuDebugPrint("AtCommand: read Seria port COM%d (fd = %d) failure...\n", pGprsSerialPortConfig->id, fd);
		return FAILURE;
	}
	buf[readlen] = 0;
	HcuDebugPrint("AtCommand: RECV(HEX)(Len = %d): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", readlen, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], readlen);

	if(0 == readlen)
	{
		*ReplyCnt = NumOfLine;
		return SUCCESS;
	}

	for(cnt = 0; cnt < readlen; cnt++)
	{
		if( (0x0D == buf[cnt]) && (0x0A == buf[cnt+1]) )
		{

			/* End of the current line via \0 */
			ReplyStr[(NumOfLine * 1024) + IdxOfCharInOneLine] = 0x00;

			/* Point to next line */
			NumOfLine = NumOfLine + 1;

			if(8 == NumOfLine) break;

			/* reset index of char in line to 0 */
			IdxOfCharInOneLine = 0;

			/* skip to cha after 0x0D 0x0A */
			cnt = cnt + 1;
		}
		else
		{
			/* Copy each of the char */
			ReplyStr[(NumOfLine * 1024) + IdxOfCharInOneLine] = buf[cnt];
			IdxOfCharInOneLine = IdxOfCharInOneLine + 1;
		}

	}

	for(i = 0; i<NumOfLine; i++)
	{
		HcuDebugPrint("AtCommand: AT RECV (Line #%02d): %s\n", i, (ReplyStr + (i * 1024)));
	}

	*ReplyCnt = NumOfLine;

	return SUCCESS;

}


void ExeConsoleCmd(const char *cmd, char *result)
{
    char buf_ps[8192];
    char ps[1024]={0};
    FILE *ptr;

    strcpy(ps, cmd);

    if((ptr = popen(ps, "r")) != NULL)
    {
        while(fgets(buf_ps, 1024, ptr) != NULL)
        {

        	strcat(result, buf_ps);

        	if(strlen(result)>8192)
        		break;
        }

        pclose(ptr);

        ptr = NULL;
    }
    else
    {
        printf("popen %s error\n", ps);
    }
}

void SetSystemTime(char *t)
{
	HcuDebugPrint("SetSystemTime: Set OS System Time to %s\n", t);
	return;
}





