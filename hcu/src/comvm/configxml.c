/*
 * configxml.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "configxml.h"
#include "trace.h"
#include "../l2frame/apigprs.h"

//static unsigned char traceFileName[] = __FILE__;

//change the ABC..Z->abc...z
/*char tolower(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return c + 'a' - 'A';
    }
    else
    {
        return c;
    }
}
*/
UINT32 htoi(char s[])/*change the string to the number*/
{
    UINT32 i;
    UINT32 n = 0;
    //UINT32 len = 0;
    if (s[0] == '0' && (s[1]=='x' || s[1]=='X')) //判断是否有前导0x或者0X
    {
        i = 2;
    }
    else
    {
        i = 0;
    }

    for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z'); i++)
    {
        if (tolower(s[i]) > '9')
        {
            //n = (UINT32)16 * n + (UINT32)(10 + tolower(s[i]) - 'a');
            n = (n << 4) | (10 + tolower(s[i]) - 'a');
        }
        else
        {
            //n = (UINT32)16 * n + (UINT32)(tolower(s[i]) - '0');
            n = (n << 4) | (tolower(s[i]) - '0');
        }
    }
    return n;
}

/*
 *
 *
 *
 */
UINT32 parseDoc(xmlDocPtr doc, xmlNodePtr cur, char *FatherElt, char *ChildElt, char *out)
{

	xmlChar *Elt;
	int Found = HCU_LOGIC_FALSE;

	if ( (NULL == FatherElt) || (NULL==ChildElt) || (NULL==out) )
	{
		HcuErrorPrint("parseDoc: invalid input pointer, return.\n");
		return FAILURE;
	}

	cur = cur->xmlChildrenNode;
	while (cur != NULL)
	{
		//HcuDebugPrint("parseDoc: search cur = %s\n", (char *)cur->name);

		if(HCU_LOGIC_TRUE == Found)
			break;

		if ((!xmlStrcmp(cur->name, (const xmlChar *)FatherElt)))
		{
			cur = cur->xmlChildrenNode;
			while (cur != NULL)
			{
				//HcuDebugPrint("parseDoc: search cur = %s\n", (char *)cur->name);
				if((!xmlStrcmp(cur->name, (const xmlChar *)ChildElt)))
				{
					Elt=xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
					//HcuDebugPrint("parseDoc: The element is: %s, for tree %s:%s\n", (char *)Elt, FatherElt, ChildElt);
					strcpy(out, (char *)Elt);
					//HcuDebugPrint("parseDoc: The element is: %s, for tree %s:%s\n", (char *)out, FatherElt, ChildElt);
					xmlFree(Elt);
					//HcuDebugPrint("parseDoc: The element is: %s, for tree %s:%s\n", (char *)out, FatherElt, ChildElt);
					Found = HCU_LOGIC_TRUE;
					break;
					//xmlFree(Elt);
				}
				cur = cur->next;
			}
			//xmlFree(Elt);
		}
		cur = cur->next;
	}
	//xmlFreeDoc(doc);

	//HcuDebugPrint("parseDoc: exit search.\n");

	if(HCU_LOGIC_TRUE == Found)
	{
		return SUCCESS;
	}
	else
	{
		return FAILURE;
	}
}
/*read the xml's 3D element*/
UINT32 parseDoc3(xmlDocPtr doc, xmlNodePtr cur, char *FatherElt, char *ChildElt, char *GrandchildrenElt, char *out)
{

	xmlChar *Elt;
	int Found = HCU_LOGIC_FALSE;

	cur = cur->xmlChildrenNode;

	if ( (NULL == FatherElt) || (NULL==ChildElt) || (NULL==GrandchildrenElt) || (NULL==out) )
	{
		HcuErrorPrint("parseDoc3: invalid input pointer, return.\n");
		return FAILURE;
	}

	while (NULL != cur)
	{
		if ((!xmlStrcmp(cur->name, (const xmlChar *)FatherElt)))
		{
			cur = cur->xmlChildrenNode;
			while (NULL != cur)
			{
				if(1 == Found)
					break;

				if((!xmlStrcmp(cur->name, (const xmlChar *)ChildElt)))
				{
					cur = cur->xmlChildrenNode;
					while(NULL != cur)
					{
						if((!xmlStrcmp(cur->name, (const xmlChar *)GrandchildrenElt)))
						{
							Elt = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);

							//HcuDebugPrint("parseDoc3: The element is: %s, for tree %s:%s:%s\n", (char *)Elt, FatherElt, ChildElt, GrandchildrenElt);
							/* Get the output */
							strcpy(out, (char *)Elt);
							xmlFree(Elt);
							Found = HCU_LOGIC_TRUE;
							break;
						}
						cur = cur->next;
					}
					//Elt=xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
					//printf("the element is: %s\n", (char *)Elt);
					//TheElementOfSearch = Elt;
					//printf("the element is: %s\n", (char *)TheElementOfSearch);
					//break;
					//xmlFree(Elt);
				}
				cur = cur->next;
			}
			//xmlFree(Elt);
		}
		cur = cur->next;
	}
	//xmlFreeDoc(doc);

	if(HCU_LOGIC_TRUE == Found)
	{
		return SUCCESS;
	}
	else
	{
		return FAILURE;
	}

}
/*apart the 64 address to the two 32 UNIT32*/
void ApartAddress(char *temp, char *stringAdd1, char *stringAdd2)
{
	char string[1024];
	int lenofthestring = 0;

	if((NULL == temp) || (NULL == stringAdd1) || (NULL == stringAdd2))
	{
		HcuErrorPrint("ApartAddress: Invalid parameter, return.\n");
		return;
	}
	strcpy(string, (char *)temp);
	//HcuDebugPrint("ApartAddress: The string is %s\n", string);
	lenofthestring=strlen(string);
	//HcuDebugPrint("ApartAddress: the len of the string is %d\n", lenofthestring);
	memcpy(stringAdd1, string, 10);
	stringAdd1[10]='\0';
	//HcuDebugPrint("ApartAddress: The stringAdd1 is %s.\n", stringAdd1);

	stringAdd2[0] = '0';
	stringAdd2[1] = 'x';
	memcpy(stringAdd2+2, string+10, 8);
	stringAdd2[10]='\0';
	//HcuDebugPrint("ApartAddress: stringAdd2 is %s.\n", stringAdd2);
}
/*
 *
 * read the xml's element
 *
 *
 */
UINT32 AnalysisOfXml(char *docv)
{

	xmlDocPtr 	doc;
	xmlNodePtr 	cur;
	char string[19];
	char stringAdd1[11];
	char stringAdd2[11]; /*deposit the 64 address*/

	char out[1024];
	UINT32 ret;
	//UINT32 SensorId = 0;
	char StrSensorId[64];
	UINT32 i;

	/* open xml file */
	doc = xmlParseFile(docv);
	if ( NULL == doc )
	{
		HcuErrorPrint("AnalysisOfXml: Xml file %s can not be open, return.\n", docv);
		return FAILURE;
	}

	/* Get to XML root element */
	cur = xmlDocGetRootElement(doc);
	if (NULL == cur)
	{
		HcuErrorPrint("AnalysisOfXml: Xml file %s can not be parsed for root element, return.\n", docv);
		xmlFreeDoc(doc);
		return FAILURE;
	}

	/* Check if the root element named as <hcu> */
	if (xmlStrcmp(cur->name, (const xmlChar *) "hcu"))
	{
		HcuErrorPrint("AnalysisOfXml: Xml file %s document of the wrong type, root node != <hcu>, return.\n", docv);
		xmlFreeDoc(doc);
		return FAILURE;
	}

	/* ============================================================================ */
	/*init the  SerialPortZigbee*/
	ret = parseDoc(doc, cur, "SerialPortZigbee", "nSpeed", out);
	if (SUCCESS == ret)
	{
		gSerialPortZb.nSpeed = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SerialPortZigbee: nSpeed = %d.\n", gSerialPortZb.nSpeed);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortZigbee: nSpeed failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortZigbee", "id", out);
	if (SUCCESS == ret)
	{
		gSerialPortZb.id = htoi((char *)out);
		HcuDebugPrint("AnalysisOfXml: SerialPortZigbee: COM port id = 0x%02X.\n", gSerialPortZb.id);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortZigbee: COM port id failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortZigbee", "nBits", out);
	if (SUCCESS == ret)
	{
		gSerialPortZb.nBits = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SerialPortZigbee: nBits = %d.\n", gSerialPortZb.nBits);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortZigbee: nBits failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortZigbee", "nEvent", out);
	if (SUCCESS == ret)
	{
		gSerialPortZb.nEvent = out[0];
		HcuDebugPrint("AnalysisOfXml: SerialPortZigbee: nEvent = %c.\n", gSerialPortZb.nEvent);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortZigbee: nEvent failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortZigbee", "nStop", out);
	if (SUCCESS == ret)
	{
		gSerialPortZb.nStop=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SerialPortZigbee: nStop = %d.\n", gSerialPortZb.nStop);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortZigbee: nStop failure\n");
		return FAILURE;
	}

	/* gSerialPortZb.fd = 0xFFFF; */

	/* ============================================================================ */
	/*init the  SerialPortGprs*/
	ret = parseDoc(doc, cur, "SerialPortGPRS", "nSpeed", out);
	if (SUCCESS == ret)
	{
		gSerialPortGprs.nSpeed = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SerialPortGPRS: nSpeed = %d.\n", gSerialPortGprs.nSpeed);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortGPRS: nSpeed failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortGPRS", "id", out);
	if (SUCCESS == ret)
	{
		gSerialPortGprs.id = htoi((char *)out);
		HcuDebugPrint("AnalysisOfXml: SerialPortGPRS: COM port id = 0x%02X.\n", gSerialPortGprs.id);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortGPRS: COM port id failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortGPRS", "nBits", out);
	if (SUCCESS == ret)
	{
		gSerialPortGprs.nBits = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SerialPortGPRS: nBits = %d.\n", gSerialPortGprs.nBits);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortGPRS: nBits failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortGPRS", "nEvent", out);
	if (SUCCESS == ret)
	{
		gSerialPortGprs.nEvent = out[0];
		HcuDebugPrint("AnalysisOfXml: SerialPortGPRS: nEvent = %c.\n", gSerialPortGprs.nEvent);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortGPRS: nEvent failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SerialPortGPRS", "nStop", out);
	if (SUCCESS == ret)
	{
		gSerialPortGprs.nStop=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SerialPortGPRS: nStop = %d.\n", gSerialPortGprs.nStop);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SerialPortGPRS: nStop failure\n");
		return FAILURE;
	}

	/* gSerialPortGprs.fd = 0xFFFF; */

	/* ============================================================================ */
	/* Initialize Frame Checking */
	ret = parseDoc(doc, cur, "FrameSetting", "IsFrameFcsCheckEnable", out);
	if (SUCCESS == ret)
	{
		gFcsEnable = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: FrameSetting: IsFrameFcsCheckEnable = %d.\n", gFcsEnable);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read FrameSetting: IsFrameFcsCheckEnable failure\n");
		return FAILURE;
	}

	/* ============================================================================ */
	/*init the  SensorMeasurementBehavior*/
	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "SerialVtime", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.SerialVtime=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: SerialVtime = %d.\n", gHcuMeasureBehavior.SerialVtime);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Read SensorMeasurementBehavior: SerialVtime failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "SerialVmin", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.SerialVmin=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: SerialVmin = %d.\n", gHcuMeasureBehavior.SerialVmin);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: SerialVmin failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "DelayBeforeReadMs", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.DelayBeforeRead=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: DelayBeforeReadMs = %d.\n", gHcuMeasureBehavior.DelayBeforeRead);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: DelayBeforeReadMs failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "DelayAfterReadMs", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.DelayAfterRead=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: DelayAfterReadMs = %d.\n", gHcuMeasureBehavior.DelayAfterRead);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: DelayAfterReadMs failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "SerialPollItvlSec", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.SerialPollItvlSec = (time_t)strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: SerialPollItvlSec = %d.\n", gHcuMeasureBehavior.SerialPollItvlSec);
	}/*the  style of the SerialPollItvlSec is wrong*/
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: SerialPollItvlSec failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "SerialPollItvlnSec", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.SerialPollItvlnSec=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: SerialPollItvlnSec = %d.\n", gHcuMeasureBehavior.SerialPollItvlnSec);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: SerialPollItvlnSec failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "NumOfPollingLoop", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.NumOfPollingLoop=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: NumOfPollingLoop = %d.\n", gHcuMeasureBehavior.NumOfPollingLoop);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: NumOfPollingLoop failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "ServerReportInverval", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.SocketInterval=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: ServerReportInverval = %d.\n", gHcuMeasureBehavior.SocketInterval);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: ServerReportInverval failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "SensorMeasurementBehavior", "MaxCounterTicksNetworkDown", out);
	if (SUCCESS == ret)
	{
		gHcuMeasureBehavior.MaxCounterTicksNetworkDown=strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: SensorMeasurementBehavior: MaxCounterTicksNetworkDown = %d.\n", gHcuMeasureBehavior.MaxCounterTicksNetworkDown);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: SensorMeasurementBehavior: MaxCounterTicksNetworkDown failure\n");
		return FAILURE;
	}

	/* ============================================================================ */
	/* Start to read 16 sensors */
	for (i = 0; i < 16; i++)
	{
		/* Get name string */
		sprintf(StrSensorId, "SensorId%03d", i);

		ret = parseDoc3(doc, cur, "SensorsConfiguration", StrSensorId, "NodeSensorType", out);
		if (SUCCESS == ret)
		{
			gZbSensors.Sensor[i].NodeSensorType = htoi((char *)out);
			HcuDebugPrint("AnalysisOfXml: gZbSensors.Sensor[%d].NodeSensorType = %0X.\n", i, gZbSensors.Sensor[i].NodeSensorType);
		}
		else
		{
			HcuErrorPrint("AnalysisOfXml: gZbSensors.Sensor[%d].NodeSensorType failure.\n", i);
			return FAILURE;
		}

		ret = parseDoc3(doc, cur, "SensorsConfiguration", StrSensorId, "NodeZbIeeeAddress", out);
		if (SUCCESS == ret)
		{
			strcpy(string, (char *)out);

			ApartAddress(string, stringAdd1, stringAdd2);
			HcuDebugPrint("AnalysisOfXml: To save address of: %s, %s.\n", stringAdd1, stringAdd2);
			gZbSensors.Sensor[i].NodeZbIeeeAddress[0] = htoi(stringAdd1);
			HcuDebugPrint("AnalysisOfXml: gZbSensors.Sensor[%d].NodeZbIeeeAddress[0] is %08X.\n", i, gZbSensors.Sensor[i].NodeZbIeeeAddress[0]);
			gZbSensors.Sensor[i].NodeZbIeeeAddress[1] = htoi(stringAdd2);
			HcuDebugPrint("AnalysisOfXml: gZbSensors.Sensor[%d].NodeZbIeeeAddress[1] is %08X.\n", i, gZbSensors.Sensor[i].NodeZbIeeeAddress[1]);
		}
		else
		{
			HcuErrorPrint("AnalysisOfXml: gZbSensors.Sensor[%d].NodeZbIeeeAddress failure\n", i);
			return FAILURE;
		}
	}

	/* ============================================================================ */
	/* GPRS config */
	ret = parseDoc(doc, cur, "Gprs", "IsGprsEnabled", out);
	if (SUCCESS == ret)
	{
		gGprsPppdConf.gUseGprsOrNot = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: Gprs: IsGprsEnabled = %d.\n", gGprsPppdConf.gUseGprsOrNot);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: IsGprsEnabled failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "GprsPppdWaitSec", out);
	if (SUCCESS == ret)
	{
		gGprsPppdConf.GprsPppdWaitSec = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: Gprs: GprsPppdWaitSec = %d.\n", gGprsPppdConf.GprsPppdWaitSec);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: GprsPppdWaitSec failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "MaxGprsCheckTimes", out);
	if (SUCCESS == ret)
	{
		gGprsPppdConf.MaxGprsCheckTimes = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: Gprs: MaxGprsCheckTimes = %d.\n", gGprsPppdConf.MaxGprsCheckTimes);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: MaxGprsCheckTimes failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "MaxGprsResetTimes", out);
	if (SUCCESS == ret)
	{
		gGprsPppdConf.MaxGprsResetTimes = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: Gprs: MaxGprsResetTimes = %d.\n", gGprsPppdConf.MaxGprsResetTimes);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: MaxGprsResetTimes failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "HwResetPortOp1", out);
	if (SUCCESS == ret)
	{
		strcpy(&gGprsPppdConf.HwResetPortOp1[0], (char *)out);
		HcuDebugPrint("AnalysisOfXml: Gprs: HwResetPortOp1 = %s.\n", gGprsPppdConf.HwResetPortOp1);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: HwResetPortOp1 failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "HwResetPortOp1DelayMsec", out);
	if (SUCCESS == ret)
	{
		gGprsPppdConf.HwResetPortOp1DelayMsec = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: Gprs: HwResetPortOp1DelayMsec = %d.\n", gGprsPppdConf.HwResetPortOp1DelayMsec);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: HwResetPortOp1DelayMsec failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "HwResetPortOp2", out);
	if (SUCCESS == ret)
	{
		strcpy(&gGprsPppdConf.HwResetPortOp2[0], (char *)out);
		HcuDebugPrint("AnalysisOfXml: Gprs: HwResetPortOp2 = %s.\n", gGprsPppdConf.HwResetPortOp2);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: HwResetPortOp2 failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "HwResetPortOp2DelayMsec", out);
	if (SUCCESS == ret)
	{
		gGprsPppdConf.HwResetPortOp2DelayMsec = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: Gprs: HwResetPortOp2DelayMsec = %d.\n", gGprsPppdConf.HwResetPortOp2DelayMsec);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: HwResetPortOp2DelayMsec failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "SyncSysTimeFromGprs", out);
	if (SUCCESS == ret)
	{
		gGprsPppdConf.SyncSysTimeFromGprs = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: Gprs: SyncSysTimeFromGprs = %d.\n", gGprsPppdConf.SyncSysTimeFromGprs);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: SyncSysTimeFromGprs failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "Gprs", "PppConfigFileName", out);
	if (SUCCESS == ret)
	{
		strcpy(&gGprsPppdConf.PppConfigFileName[0], (char *)out);
		HcuDebugPrint("AnalysisOfXml: Gprs: SyncSysTimeFromGprs = %s.\n", gGprsPppdConf.PppConfigFileName);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Gprs: PppConfigFileName failure\n");
		return FAILURE;
	}
	/* ============================================================================ */
	/* Server Socket config */
	ret = parseDoc(doc, cur, "ServerConfig", "ip", out);
	if (SUCCESS == ret)
	{
		strcpy(gSocketPort.ServerIpAddress, (char *)out);
		HcuDebugPrint("AnalysisOfXml: gSocketPort.ServerIpAddress = %s.\n", gSocketPort.ServerIpAddress);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: gSocketPort.ServerIpAddress failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "ServerConfig", "port", out);
	if (SUCCESS == ret)
	{
		gSocketPort.ServerPort = strtol((char *)out, NULL, 10);
		HcuDebugPrint("AnalysisOfXml: gSocketPort.ServerPort = %d.\n", gSocketPort.ServerPort);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: gSocketPort.ServerPort failure\n");
		return FAILURE;
	}

	/* ============================================================================ */
	/* Server Header */
	ret = parseDoc(doc, cur, "MsgSeverDataSegHeader", "ST", out);
	if (SUCCESS == ret)
	{
		strcpy(gMsgSeverDataSegHeader.ST, (char *)out);
		HcuDebugPrint("AnalysisOfXml: gMsgSeverDataSegHeader.ST = %s.\n", gMsgSeverDataSegHeader.ST);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: gMsgSeverDataSegHeader.ST failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "MsgSeverDataSegHeader", "CN", out);
	if (SUCCESS == ret)
	{
		strcpy(gMsgSeverDataSegHeader.CN, (char *)out);
		HcuDebugPrint("AnalysisOfXml: gMsgSeverDataSegHeader.CN = %s.\n", gMsgSeverDataSegHeader.CN);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: gMsgSeverDataSegHeader.CN failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "MsgSeverDataSegHeader", "PW", out);
	if (SUCCESS == ret)
	{
		strcpy(gMsgSeverDataSegHeader.PW, (char *)out);
		HcuDebugPrint("AnalysisOfXml: gMsgSeverDataSegHeader.PW = %s.\n", gMsgSeverDataSegHeader.PW);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: gMsgSeverDataSegHeader.MN failure\n");
		return FAILURE;
	}

	ret = parseDoc(doc, cur, "MsgSeverDataSegHeader", "MN", out);
	if (SUCCESS == ret)
	{
		strcpy(gMsgSeverDataSegHeader.MN, (char *)out);
		HcuDebugPrint("AnalysisOfXml: gMsgSeverDataSegHeader.MN = %s.\n", gMsgSeverDataSegHeader.MN);
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: gMsgSeverDataSegHeader.MN failure\n");
		return FAILURE;
	}

	/* ============================================================================ */
	/* Trace config */
	ret = parseDoc(doc, cur, "Trace", "ToConsoleEnable", out);
	if (SUCCESS == ret)
	{
		outputOnCrt = strtol((char *)out, NULL, 10);
		if( HCU_LOGIC_FALSE == outputOnCrt)
		{
			outputOnCrt = HCU_LOGIC_TRUE;
			HcuDebugPrint("AnalysisOfXml: Trace: ToConsoleEnable = 0, Even though the trace is set to close, still open temperally during HCU startup ...\n");
			TraceWillBeDisplayed = HCU_LOGIC_FALSE;
		}
		else
		{
			HcuDebugPrint("AnalysisOfXml: Trace: ToConsoleEnable = %d.\n", outputOnCrt);
		}
	}
	else
	{
		HcuErrorPrint("AnalysisOfXml: Trace: ToConsoleEnable failure\n");
		return FAILURE;
	}

	/* FREE THE FILE POINTER */
	xmlFreeDoc(doc);
	HcuDebugPrint("AnalysisOfXml: Xml file handle (%x) freed, read complete.\n", doc);
	return SUCCESS;
}
