/*
 * sysinfo.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "sysinfo.h"
#include "trace.h"

#define MAXINTERFACES 20
/* FOR LIST OF INTERFACE */

#define PROC_DIRECTORY "/proc/"
#define CASE_SENSITIVE    1
#define CASE_INSENSITIVE  0
#define EXACT_MATCH       1
#define INEXACT_MATCH     0

//static unsigned char traceFileName[] = __FILE__;

int IsNumeric(const char* ccharptr_CharacterList)
{
    for ( ; *ccharptr_CharacterList; ccharptr_CharacterList++)
        if (*ccharptr_CharacterList < '0' || *ccharptr_CharacterList > '9')
            return 0; // false
    return 1; // true
}

int strcmp_Wrapper(const char *s1, const char *s2, int intCaseSensitive)
{
    if (intCaseSensitive)
        return !strcmp(s1, s2);
    else
        return !strcasecmp(s1, s2);
}

int strstr_Wrapper(const char* haystack, const char* needle, int intCaseSensitive)
{
    if (intCaseSensitive)
        return (int) strstr(haystack, needle);
    else
        return (int) strcasestr(haystack, needle);
}

pid_t GetPIDbyName_implements(const char* cchrptr_ProcessName, int intCaseSensitiveness, int intExactMatch)
{
    char chrarry_CommandLinePath[100]  ;
    char chrarry_NameOfProcess[300]  ;
    char* chrptr_StringToCompare = NULL ;
    pid_t pid_ProcessIdentifier = (pid_t) -1 ;
    struct dirent* de_DirEntity = NULL ;
    DIR* dir_proc = NULL ;

    int (*CompareFunction) (const char*, const char*, int) ;

    if (intExactMatch)
        CompareFunction = &strcmp_Wrapper;
    else
        CompareFunction = &strstr_Wrapper;


    dir_proc = opendir(PROC_DIRECTORY) ;
    if (dir_proc == NULL)
    {
        perror("Couldn't open the " PROC_DIRECTORY " directory") ;
        return (pid_t) -2 ;
    }

    // Loop while not NULL
    while ( (de_DirEntity = readdir(dir_proc)) )
    {
        if (de_DirEntity->d_type == DT_DIR)
        {
            if (IsNumeric(de_DirEntity->d_name))
            {
                strcpy(chrarry_CommandLinePath, PROC_DIRECTORY) ;
                strcat(chrarry_CommandLinePath, de_DirEntity->d_name) ;
                strcat(chrarry_CommandLinePath, "/cmdline") ;
                FILE* fd_CmdLineFile = fopen (chrarry_CommandLinePath, "rt") ;  // open the file for reading text
                if (fd_CmdLineFile)
                {
                    fscanf(fd_CmdLineFile, "%s", chrarry_NameOfProcess) ; // read from /proc/<NR>/cmdline
                    fclose(fd_CmdLineFile);  // close the file prior to exiting the routine

                    if (strrchr(chrarry_NameOfProcess, '/'))
                        chrptr_StringToCompare = strrchr(chrarry_NameOfProcess, '/') +1 ;
                    else
                        chrptr_StringToCompare = chrarry_NameOfProcess ;

                    //printf("Process name: %s\n", chrarry_NameOfProcess);
                    //printf("Pure Process name: %s\n", chrptr_StringToCompare );

                    if ( CompareFunction(chrptr_StringToCompare, cchrptr_ProcessName, intCaseSensitiveness) )
                    {
                        pid_ProcessIdentifier = (pid_t) atoi(de_DirEntity->d_name) ;
                        closedir(dir_proc) ;
                        return pid_ProcessIdentifier ;
                    }
                }
            }
        }
    }
    closedir(dir_proc) ;
    return pid_ProcessIdentifier ;
}

// C cannot overload functions - fixed
pid_t GetPIDbyName_Wrapper(const char* cchrptr_ProcessName, ... )
    {
        int intTempArgument ;
        int intInputArguments[2] ;
        // intInputArguments[0] = 0 ;
        // intInputArguments[1] = 0 ;
        memset(intInputArguments, 0, sizeof(intInputArguments) ) ;
        int intInputIndex ;
        va_list argptr;

        va_start( argptr, cchrptr_ProcessName );
        for (intInputIndex = 0;  (intTempArgument = va_arg( argptr, int )) != 15; ++intInputIndex)
        {
            intInputArguments[intInputIndex] = intTempArgument ;
        }
        va_end( argptr );
        return GetPIDbyName_implements(cchrptr_ProcessName, intInputArguments[0], intInputArguments[1]);
    }

#define GetPIDbyName(ProcessName,...) GetPIDbyName_Wrapper(ProcessName, ##__VA_ARGS__, (int) 15)


void diep(char *s)
{
    perror(s);
    exit(1);
}


UINT32 HcuGetSysInfo(SysInfo_t *si)
{

    /* HW CPU Information */
    FILE *cmdline = fopen("/proc/cpuinfo", "rb");
    char *arg = NULL;
    size_t size = 0;
    while(getline(&arg, &size, cmdline) != -1)
    {
    	HcuDebugPrint("SYSINFO: CPU: %s", arg);
    	if(strstr(arg, "Processor"))
    	{
    		strcpy(si->hw.cpu_processor, arg);
    		HcuDebugPrint("SYSINFO: CPU: %s", arg);
    	}
    	free(arg);
    	arg = NULL;
    }
    fclose(cmdline);

    /* OS Information */
    cmdline = fopen("/proc/version", "rb");
    while(getline(&arg, &size, cmdline) != -1)
    {
    	HcuDebugPrint("SYSINFO: OS VERSION: %s", arg);
    	strcpy(si->os.version_signature, arg);
    	free(arg);
    	arg = NULL;
    }
    fclose(cmdline);

	/* GET PPP PID */
	pid_t pid = GetPIDbyName("ppp"); // If -1 = not found, if -2 = proc fs access error
    si->net.ppppid = pid;
    if(pid < 0)
    {
    	HcuDebugPrint("SYSINFO: PID of PPPD = %d (0x%X), No pppd launched ...\n", si->net.ppppid, si->net.ppppid);
    }
    else
    {
    	HcuDebugPrint("SYSINFO: PID of PPPD = %d (0x%X), pppd already launched ...\n", si->net.ppppid, si->net.ppppid);
    }

    /* GET NETINTERFACE */
    int sock;
    struct ifconf ifconf;
    struct ifreq ifreq[MAXINTERFACES];
    int interfaces;
    int i;

    // Create a socket or return an error.
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
    	HcuErrorPrint("SYSINFO: socket(AF_INET, SOCK_STREAM, 0) < 0, Error.\n");
    	return FAILURE;
    }

    // Point ifconf's ifc_buf to our array of interface ifreqs.
    ifconf.ifc_buf = (char *) ifreq;

    // Set ifconf's ifc_len to the length of our array of interface ifreqs.
    ifconf.ifc_len = sizeof ifreq;

    //  Populate ifconf.ifc_buf (ifreq) with a list of interface names and addresses.
    if (ioctl(sock, SIOCGIFCONF, &ifconf) == -1)
    {
      	HcuErrorPrint("SYSINFO: ioctl(sock, SIOCGIFCONF, &ifconf) == -1, Error.\n");
       	return FAILURE;
    }

    // Divide the length of the interface list by the size of each entry.
    // This gives us the number of interfaces on the system.
    interfaces = ifconf.ifc_len / sizeof(ifreq[0]);
    si->net.itfnum = interfaces;

    // Print a heading that includes the total # of interfaces.
    //printf("IF(%d)\tIP\n", interfaces);

    // Loop through the array of interfaces, printing each one's name and IP.
    HcuDebugPrint("SYSINFO: NETWORK: [ITF]\t\t[IP]\n");

    for (i = 0; i < interfaces; i++) {
        char ip[INET_ADDRSTRLEN];
        struct sockaddr_in *address = (struct sockaddr_in *) &ifreq[i].ifr_addr;

        // Convert the binary IP address into a readable string.
        if (!inet_ntop(AF_INET, &address->sin_addr, ip, sizeof(ip)))
            diep("inet_ntop");

        //printf("%s\t%s\n", ifreq[i].ifr_name, ip);
        strcpy(si->net.itfname[i * MAX_SYS_INFO_STRING_LENGTH], ifreq[i].ifr_name);
        strcpy(si->net.itfip[i * MAX_SYS_INFO_STRING_LENGTH], ip);
        HcuDebugPrint("SYSINFO: NETWORK: [%s]\t\t[%s]\n", si->net.itfname[i * MAX_SYS_INFO_STRING_LENGTH], si->net.itfip[i * MAX_SYS_INFO_STRING_LENGTH]);

    }
    close(sock);


    return SUCCESS;
}

UINT32 IsNetItfExist(char *itfname)
{
    /* GET NETINTERFACE */
    int sock;
    struct ifconf ifconf;
    struct ifreq ifreq[MAXINTERFACES];
    int interfaces;
    int i;

    if(NULL == itfname)
    {
    	HcuErrorPrint("IsNetItfExist: Invalid input of itfname.\n");
    	return FAILURE;
    }

    // Create a socket or return an error.
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
    	HcuErrorPrint("IsNetItfExist: socket(AF_INET, SOCK_STREAM, 0) < 0, Error.\n");
    	return FAILURE;
    }

    // Point ifconf's ifc_buf to our array of interface ifreqs.
    ifconf.ifc_buf = (char *) ifreq;

    // Set ifconf's ifc_len to the length of our array of interface ifreqs.
    ifconf.ifc_len = sizeof ifreq;

    //  Populate ifconf.ifc_buf (ifreq) with a list of interface names and addresses.
    if (ioctl(sock, SIOCGIFCONF, &ifconf) == -1)
    {
      	HcuErrorPrint("IsNetItfExist: ioctl(sock, SIOCGIFCONF, &ifconf) == -1, Error.\n");
       	return FAILURE;
    }

    // Divide the length of the interface list by the size of each entry.
    // This gives us the number of interfaces on the system.
    interfaces = ifconf.ifc_len / sizeof(ifreq[0]);
    //si->net.itfnum = interfaces;

    // Print a heading that includes the total # of interfaces.
    //printf("IF(%d)\tIP\n", interfaces);

    // Loop through the array of interfaces, printing each one's name and IP.
    HcuDebugPrint("IsNetItfExist: Start to seach for interface [%s] ...\n", itfname);

    for (i = 0; i < interfaces; i++) {
        char ip[INET_ADDRSTRLEN];
        struct sockaddr_in *address = (struct sockaddr_in *) &ifreq[i].ifr_addr;

        // Convert the binary IP address into a readable string.
        if (!inet_ntop(AF_INET, &address->sin_addr, ip, sizeof(ip)))
            diep("inet_ntop");

        //printf("%s\t%s\n", ifreq[i].ifr_name, ip);
        if (!strcmp(ifreq[i].ifr_name, itfname))
        {
        	HcuDebugPrint("IsNetItfExist: Network interface [%s] found, with ip address of [%s]\n", itfname, ip);
            close(sock);
            return SUCCESS;
        }
    }

    /* Close Socket */
    close(sock);

    HcuDebugPrint("IsNetItfExist: Network Interface [%s] not found.\n", itfname);
    return FAILURE;

}


UINT32 IsGprsPppdOk()
{

	/* GET PPP PID */
	pid_t pid = GetPIDbyName("ppp"); // If -1 = not found, if -2 = proc fs access error
	//char killpppd[64];
	//si->net.ppppid = pid;
    if(pid < 0)
    {
    	HcuDebugPrint("IsGprsPppdOk: PID of PPPD = %d (0x%X), No pppd launched ...\n", pid, pid);
    	return FAILURE;
    }
    else
    {
    	HcuDebugPrint("IsGprsPppdOk: PID of PPPD = %d (0x%X), pppd already launched ...\n", pid, pid);
    }

    /* GET NETINTERFACE */
    int sock;
    struct ifconf ifconf;
    struct ifreq ifreq[MAXINTERFACES];
    int interfaces;
    int i;

    // Create a socket or return an error.
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
    	HcuErrorPrint("IsGprsPppdOk: socket(AF_INET, SOCK_STREAM, 0) < 0, Error.\n");
    	return FAILURE;
    }

    // Point ifconf's ifc_buf to our array of interface ifreqs.
    ifconf.ifc_buf = (char *) ifreq;

    // Set ifconf's ifc_len to the length of our array of interface ifreqs.
    ifconf.ifc_len = sizeof ifreq;

    //  Populate ifconf.ifc_buf (ifreq) with a list of interface names and addresses.
    if (ioctl(sock, SIOCGIFCONF, &ifconf) == -1)
    {
      	HcuErrorPrint("IsGprsPppdOk: ioctl(sock, SIOCGIFCONF, &ifconf) == -1, Error.\n");
       	return FAILURE;
    }

    // Divide the length of the interface list by the size of each entry.
    // This gives us the number of interfaces on the system.
    interfaces = ifconf.ifc_len / sizeof(ifreq[0]);
    //si->net.itfnum = interfaces;

    // Print a heading that includes the total # of interfaces.
    //printf("IF(%d)\tIP\n", interfaces);

    // Loop through the array of interfaces, printing each one's name and IP.
    HcuDebugPrint("IsGprsPppdOk: Start to seach for ppp0 interface ...\n");

    for (i = 0; i < interfaces; i++) {
        char ip[INET_ADDRSTRLEN];
        struct sockaddr_in *address = (struct sockaddr_in *) &ifreq[i].ifr_addr;

        // Convert the binary IP address into a readable string.
        if (!inet_ntop(AF_INET, &address->sin_addr, ip, sizeof(ip)))
            diep("inet_ntop");

        //printf("%s\t%s\n", ifreq[i].ifr_name, ip);
        if (!strcmp(ifreq[i].ifr_name, "ppp0"))
        {
        	HcuDebugPrint("IsGprsPppdOk: Network Interface ppp0 found, with ip address of %s\n", ip);
            close(sock);
            return SUCCESS;
        }
    }

    /* Close Socket */
    close(sock);

    HcuDebugPrint("IsGprsPppdOk: Network Interface ppp0 not found, even though pppd process (%d) exsit ...\n", pid);
    return FAILURE;
}

void KillPppdProcess()
{
	/* GET PPP PID */
	pid_t pid = GetPIDbyName("ppp"); // If -1 = not found, if -2 = proc fs access error
	char killpppd[64];

    /* If the PID is actived, but there is no interface found, kill the process to free the serial ports */
    sprintf(killpppd, "kill -9 %d", pid);
    if(pid > 0)
    {
    	system(killpppd);
    	HcuErrorPrint("KillPppdProcess: pppd process (%d) killed by called %s.\n", pid, killpppd);
    }
    else
    {
    	HcuErrorPrint("KillPppdProcess: no active pppd, return.\n");
    	return;
    }
}



