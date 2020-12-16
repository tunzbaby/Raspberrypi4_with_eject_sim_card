#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>

/*

cc -c -O libCrt711_Test.c

cc libCrt711_Test.o  -o libCrt711_Test  ./libCrt711.so

./libCrt711_Test

*/

int bPortOpen=0;  
void *handle;
int CpuCardType; //current cpu Card type
int menu();
int port_fd;
unsigned char Rdata[256]={0};
unsigned int Rlen=0;

void OpenDeviceFunction();
void CloseDeviceFunction();
void InitDeviceFunction();
void CallEntryFunction();
void DisEntryFunction();
void EjectFunction();
void CaptureFunction();
//-------------------------------------------------------------------------------------------

//int OpenDevice(char *sPort, char* sMes)
void OpenDeviceFunction()
{
    char devicename[50]="/dev/ttyS0";
    int Reback;


    printf("Sample of the device name  \n");   
    printf("format of COM1 PORT is /dev/ttyS0  etc.\n");    
    printf("format of USB PORT 1 is /dev/ttyUSB0  etc.\n");     
    printf("Please input the device name(/dev/ttyS0):\n");
    Reback=scanf("%s",devicename);

    port_fd=open_port(devicename);
    if (port_fd>0)
    {
       printf("Open device OK,fd is:%d\n",port_fd);

       bPortOpen=1;
    }
    else
    { 
        printf("Open device Error,fd is %d\n",port_fd);
    }
}

//int InitDevice (int Option,char* sMes);
void InitDeviceFunction()
{
    unsigned char Tdata[]={0x43,0x30,0x33};
    int Resuilt=-1;
    int i;
    
    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("InitDevice OK\n"); 
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    { 
        printf("InitDevice Error,error code is %d\n",Resuilt);
    }
}

//int CloseDevice (char* sMes);
void CloseDeviceFunction()
{
    int Resuilt=-1;

    Resuilt=close_port(port_fd);
    if (Resuilt==0)
    {
       printf("CloseDevice OK\n"); 
    }
    else
    { 
        printf("CloseDevice Error,error code is %d\n",Resuilt);
    }
}


//int CallEntry(char* sMes);
void CallEntryFunction()
{
    unsigned char Tdata[]={0x43,0x33,0x30};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("CallEntry OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("CallEntry Error,error code is %d\n",Resuilt);
    }
}
//int DisEntry (char* sMes);
void DisEntryFunction()
{
    unsigned char Tdata[]={0x43,0x33,0x31};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("DisEntry OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("DisEntry Error,error code is %d\n",Resuilt);
    }
}

//int Eject(char* sMes);
void EjectFunction()
{
    unsigned char Tdata[]={0x43,0x32,0x30};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("Eject OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("Eject Error,error code is %d\n",Resuilt);
    }
}

//int Capture(char* sMes);  capture to 1#
void CaptureFunction()
{
    unsigned char Tdata[]={0x43,0x32,0x33};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("Capture OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("Capture Error,error code is %d\n",Resuilt);
    }
}


int menu()
{
	int i;
        int Reback;
	i=0;

    //CPUPowerOnFunction
  //CPUChipIoFunction
    //CPUPowerOffFunction
	while(1)
	{


		printf("Please select a menu(0~9):\n");
		printf("    ---------Communication Settings----------------------\n");
		printf("    ---------Reader Operation----------------------------\n");
        printf("    1:  OpenDevice.\n");
        printf("    2:  InitDevice.\n");
        printf("    3:  CallEntry.\n");
        printf("    4:  DisEntry.\n");
        printf("    5:  Eject.\n");
        printf("    6:  Capture.\n");
		printf("    ----------Exit -----------------------------\n");
		printf("    0:  Exit (Close the Comm. port).\n");
		Reback=scanf("%d",&i);
		printf("\n");
          	printf("You have chosen: %d\n",i);
		if (i==0)
		{
		        if (bPortOpen==1) 
		        {
			    CloseDeviceFunction();//关闭串口	
		        }    
			printf("Byebye.\n\n\n");
            //dlclose(handle);
			exit(1);
		}
        if (i<0 || i>13)
		{
			printf("Error.\n\n");
			continue;
		}
		else
		{
			return(i);
		}
	}
}

main()
{
   int s;
   if (!isatty(fileno(stdout)))
   {
      fprintf(stderr,"You ard not a termial!\n");
      exit(1);
   } 




   while(1)
    {
       s=menu();
       if (s==1)
       {
           OpenDeviceFunction();
       } 
       if (s==2)
       {
           InitDeviceFunction();
       }    
       if (s==3)
       {
           CallEntryFunction();
       }    
       if (s==4)
       {
           DisEntryFunction();
       }
       if (s==5)
       {
           EjectFunction();
       }    
       if (s==6)
       {
           CaptureFunction();
       }    

    }	
}
