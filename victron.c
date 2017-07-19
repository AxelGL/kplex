/* victron.c
 * This file is part of kplex
 * Copyright  AxelGL
 * For copying information see the file COPYING distributed with this software
 *
 * This file contains code for Victron Solar Charger. 
   It reads the data from the UART interface and converts it to a NMEA string according to the statement in the kplex.conf file. 
   Example: 
   [victron]
filename=/dev/ttyAMA0
direction=in
baud=19200
eol=rn
strict=no
#nmeastring=V,$SSMTW,C
#nmeastring=V,$IIDPT,0
#nmeastring=I,$SSMTW,C
nmeastring=P,$IIMTW,C
#nmeastring=W,$SSMTW,C
#nmeastring=W,$IIXDR,C

There is a new command nmeastring that configures the NMEA string that is transmitted. 
First value: Can be 
V: Battery Voltage 
I: Battery Current
P: Panel Voltage 
E: Energy harvest from the same day
Y: Energy harvest from yesterday 
O: Energy harvest Overall  

Next is the NMEA String that is used. Depending on the string the data is displayed in the appropriate format:
IIXDR is the default. Others an be used to show data on devices that cannot display IIXDR )which is probably the norm). 
 */

#include "kplex.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <limits.h>
#if defined  __APPLE__ || defined __NetBSD__ || defined __OpenBSD__
#include <util.h>
#elif defined __FreeBSD__
#include <libutil.h>
#else
#include <pty.h>
#endif
#include <grp.h>
#include <pwd.h>

#define DEFSERIALQSIZE 128

struct if_serial {
    int fd;
    char *slavename;            /* link to pty slave (if it exists) */
    int saved;                  /* Are stored terminal settins valid? */
    struct termios otermios;    /* To restore previous interface settings
                                 *  on exit */
};


struct victron_nmea {
     char nmeastring[7];    //
     char unit;             // Each value can have a different unit to match to receiver
};
      
char nmeadispl_temp = 0xff;
char nmeastring[7];
char nmeatype[9];   // Which nmea data  should be send
char nmeaadd = 0;   // send both nmeatypes or just one

struct victron_nmea nmeastring0;
struct victron_nmea nmeastringv;
struct victron_nmea nmeastringp;
struct victron_nmea nmeastringw;
struct victron_nmea nmeastringi;
struct victron_nmea nmeastringe;
struct victron_nmea nmeastringy;
struct victron_nmea nmeastringo;



/*
 * Cleanup interface on exit
 * Args: pointer to interface
 * Returns: Nothing
 */
void cleanup_victron(iface_t *ifa)
{
    struct if_serial *ifs = (struct if_serial *)ifa->info;

    if (!ifa->pair) {
        if (ifs->saved) {
            if (tcsetattr(ifs->fd,TCSAFLUSH,&ifs->otermios) < 0) {
                if (ifa->type != PTY || errno != EIO)
                    logwarn("Failed to restore serial line: %s",strerror(errno));
            }
        }
        if (ifs->slavename) {
            if (unlink(ifs->slavename) < 0)
                logerr(errno,"Failed to remove link %s",ifs->slavename);
            free(ifs->slavename);
        }
    }
    close(ifs->fd);
}

//returns delta of char in string
int charstr(char *string, char c) {
  int i;
  for (i=0; i< 10; i++) {
    if (*string == c) return i;  
    string++;                  
  }
  return (-1);
}

/*
 * Read from a serial interface from Victron Controller, comvert to NMEA end put into Buffer
 * Args: pointer to interface structure pointer to buffer
 * Returns: Number of bytes read, zero on error or end of file
 */
ssize_t read_victron(struct iface *ifa, char *buf)
{
  struct if_serial *ifs = (struct if_serial *) ifa->info;   // Get serial specific information from iface struct. Used to get file descriptor
  char* ptr=NULL;
  char* bufi;
  int ret=0;
  int number, amount;
  char limiter[] = "\n\t";
#define IBUFSIZE 4000
  unsigned char bufint[IBUFSIZE];     // Buffer for data from UART

  do {    // Repeat until data is there
  amount = 0;     // Amount of data read from Victron
  number = 0;     // Number of NMEA char to send to network 

  bufi = buf;
  int i = 0;
  int j = 0;
  unsigned char sum = 1;
  char nmea_set = 0;      // Special NMEA String in kplex.opt requested

            DEBUG(9,"New Start ****************");

  while (sum != 0) {  
    amount = 0;
    i = 0;
    // Initialize Buffer
    for (i=0; i< IBUFSIZE; i++)  bufint[i] = 0;

    i= 0;
    do {
        usleep(40000);
        amount += read(ifs->fd,&bufint[amount],BUFSIZE) ;   // Get data from input
        bufint [amount + 1] = 0;
           // DEBUG(9,"Data %s amount = %i ",&bufint[1], amount);
    } while (!(strstr(&bufint[0], "hecksum\t")) && amount<(IBUFSIZE - 20));    //  All we know: Last line is 0x0a, 0x0a, Checksum followed by 0x09, xx


    unsigned char *checksump;
    checksump = strstr(&bufint[0], "hecksum");
    checksump += 8;   // Point to checksumbyte

//    if (amount < (checksump-&bufint[0]))   // Did not read the actual checksum (we can only detect "checksum" as end of block, but checksum value is 1 Byte later)
//      amount += read(ifs->fd,&bufint[i*8],1) ;   // Get some more data from input following "checksum"


    i = 0;
    sum = 0;
    do {
      sum += bufint[i]; 
    } while ((&bufint[i++] != checksump) && (amount < IBUFSIZE - BUFSIZE)) ;
    if (sum != 0) DEBUG(9,"Checksum wrong = %i ",sum);
    //sum = 0;   // Ignore checksum
  }


  
  ptr = strtok(&bufint[0], limiter);    // PTR contains the next token from the data from the victron

  // Go throught the different possible defined NMEA output strings if not defined, skip
  char valid = 0;

  do {
    switch(nmeadispl_temp) {
      case 0x02:    nmeadispl_temp = 0x04; break;
      case 0x04:    nmeadispl_temp = 0x08; break;
      case 0x08:    nmeadispl_temp = 0x10; break;
      case 0x10:    nmeadispl_temp = 0x20; break;
      case 0x20:    nmeadispl_temp = 0x40; break;
      case 0x40:    nmeadispl_temp = 0x80; break;
      case 0x80:    nmeadispl_temp = 0xff; break;
      case 0xff:    nmeadispl_temp = 0x02; break;
      default:      nmeadispl_temp = 0xff;
    }
    if (nmeadispl_temp == 0x02) valid = nmeastringv.nmeastring[0];
    else if (nmeadispl_temp == 0x04) valid = nmeastringp.nmeastring[0];
    else if (nmeadispl_temp == 0x08) valid = nmeastringi.nmeastring[0];
    else if (nmeadispl_temp == 0x10) valid = nmeastringw.nmeastring[0];
    else if (nmeadispl_temp == 0x20) valid = nmeastringo.nmeastring[0];
    else if (nmeadispl_temp == 0x40) valid = nmeastringe.nmeastring[0];
    else if (nmeadispl_temp == 0x80) valid = nmeastringy.nmeastring[0];
    else if (nmeadispl_temp == 0xff) valid = nmeastring0.nmeastring[0];  // 0xff always runs once to send default $IIXDR NMEA string
  } while (valid == 0); 

      DEBUG(9,"nmeadispl_temp  %x", nmeadispl_temp);


  while(ptr!=NULL)
  {
    char marker;    //  
    if ((strcmp(ptr, "VPV")==0) && (strlen(ptr) == 3)) marker = 'P';   // Voltage Solar Panel
    else if((strcmp(ptr, "V") == 0) && (strlen(ptr) == 1))  marker = 'V'; // Voltage Battery
    else if ((strcmp(ptr, "I") == 0) && (strlen(ptr) == 1)) marker = 'I'; // Current Battery
    else if ((strcmp(ptr, "PPV")==0) && (strlen(ptr) == 3)) marker = 'W'; // Power SolarPanel
    else if ((strcmp(ptr, "H19")==0) && (strlen(ptr) == 3)) marker = 'O'; // Energy Yield overall 
    else if ((strcmp(ptr, "H20")==0) && (strlen(ptr) == 3)) marker = 'E'; // Energy Yield Today
    else if ((strcmp(ptr, "H22")==0) && (strlen(ptr) == 3)) marker = 'Y'; // Energy Yield Yesterday
    else marker = 0;


    if (((marker == 'V') && (nmeadispl_temp & 0x02)) || ((marker == 'P') && (nmeadispl_temp & 0x04)))    // handle voltage or solarvoltage values with decimal point
    {

      if ((nmeadispl_temp == 0x0ff) && (number < 5)) {    // Just started, put default NMEA string $IIXDR to target string
         strncpy(bufi, nmeastring0.nmeastring, 6);
         number += 6;
         bufi+=6;
      }
      else if ((marker == 'V') && (nmeadispl_temp ==  0x02)) {   // Special NMEAString for V
        strncpy(bufi, nmeastringv.nmeastring, 6);   
         number += 6;
         bufi+=6;
      }
      else if ((marker == 'P') && (nmeadispl_temp ==  0x04)) {   // Special defined NMEA String for P
        strncpy(bufi, nmeastringp.nmeastring, 6);  
         number += 6;
         bufi+=6;
      }

      //DEBUG(9,"bufi = %s", buf);
      ptr = strtok(NULL, limiter);   // look for TAB
      if (ptr!=NULL)  
      {              // Generate NMEA sentence and copy to buffer from mV  $IIXDR,U,x.x,V,V001,checksum
           if ((marker == 'P') && (nmeadispl_temp ==  0x0ff)) { 
             strncpy(bufi, ",P", 2);  
             bufi += 2;
             number += 2;
           }
           else if (nmeadispl_temp == 0x0ff) {
             strncpy(bufi, ",U", 2);    // Marker is V for Voltage 
             bufi += 2;
             number += 2;
           }
          
           //DEBUG(8,"Data %i: %s",number, bufi);
 
           strncpy(bufi, ",", 1);
           bufi++;
           number++;
           int post = charstr(ptr, 0x0d);   // Position of TAB marker  (end of number)
           DEBUG(9,"Post  %i marker %c ptr %s",post, marker, ptr);
           if (post == 5) {        // Number has 5 digits
             strncpy(bufi, ptr, 2);   // First two voltage digits
             bufi+=2;
             ptr+=2;
             number += 2;
           }
           else if (post == 4) { 
             strncpy(bufi, ptr, 1);   // First voltage digit
             bufi+=1;
             ptr+=1;
             number += 1;
           }
           else {    // Post == 3,2,1   Only mV Value no number before .
             *bufi = '0';   // First voltage digit
             bufi+=1;
             number += 1;
           }
           strncpy(bufi++, ".", 1);
           number += 1; 
           // ptr points to next valid value
           if (post >= 3) strncpy(bufi, ptr, 1);    // 100mV value
           else *bufi = '0'; 
           bufi+=1;
           ptr+=1;
           number += 1;
           if (nmeadispl_temp == 0x0ff) {
             if (post >= 2) strncpy(bufi, ptr, 1);  // 10mV Value
             else *bufi = '0';    // 10mV
             bufi+=1;
             ptr+=1; 
             number += 1;
             strcpy(bufi, ",V,U1");    //  Now Voltage and U001 as index 
             bufi+=5;
             //DEBUG(9,"Data to %s",buf);
             //return(24);
             number += 5;
           }
           else {    // For normal NMEA the 10 mV and 1mV are not shown and skipped
             *bufi = ','; 
             bufi++;
             number++; 
             if (marker == 'V') 
                *bufi= nmeastringv.unit;
             else
                *bufi= nmeastringp.unit;
             bufi++;
             number++; 
           }
      }
      else
      {
        bufi -= 2;
        printf("value of main battery voltage missing!\n");
      }
    }
    else if ((marker == 'I') && (nmeadispl_temp & 0x08))    // handle  "I" for Battery Current
    {
      DEBUG(9,"Handling Marker I nmeadispl_temp %x", nmeadispl_temp);
      ptr = strtok(NULL, limiter);   // look for TAB
      if (ptr!=NULL)  
      {              // Generate NMEA sentence and copy to buffer  $IIXDR,I,xx,mA,U001,
        if (nmeadispl_temp == 0x0ff) { 
           strncpy(bufi, ",I,", 3);  
           bufi+=3; 
           char i = 0;
           while ( (('0' <= *(ptr+i)) && (*(ptr+i) <= '9')) || *(ptr+i)== '-') {
            *bufi++ = *(ptr+i); 
            number++;
            i++;
           }
          strncpy(bufi, ",mA,U1",8);    //  Now mA  and U001 as index 
          bufi+=6;
          number += 9;
        }
        else {
          strncpy(bufi, nmeastringi.nmeastring, 6);  
          bufi+=6; 
          *bufi++ = ','; 
          number++;
          char i = 0;
          j = 0;
          // Value from Vitronic is in mA, switch to A
          while ( (('0' <= *(ptr+i)) && (*(ptr+i) <= '9')) || *(ptr+i)== '-') {  // How many digits ?
            i++;
          }
          DEBUG(9,"Handling Current I: %i", i);
          if (*(ptr)== '-') {*bufi++ = '-'; number++; j++;}
          if (i == 5) {*bufi++ = *(ptr+j); j++; number++;}               // 10A
          if (i == 4) {*bufi++ = *(ptr+j); j++;} else *bufi++ = '0';  // A
          number++;
          *bufi++ = '.'; 
          number++;
          if (i == 3) {*bufi++ = *(ptr+j); j++;} else {*bufi++ = '0';} // 100mA
          number++;
          //if (i == 2) {*bufi++ = *(ptr+j); j++;} else {*bufi++ = '0';} // 10mA
          //number++;
          //*bufi++ = *(ptr+j);    // mA
          //number++;
          
          *bufi++ = ','; 
          number++;
          *bufi++ = nmeastringi.unit;
          number++;
        }

      }
      else
      {
        bufi -= 2;
        printf("value of main current missing!\n");
      }
    }
    else if ((marker == 'W') && (nmeadispl_temp & 0x10))    // handle  "PPV" for Solar Panel Power
    {
      DEBUG(9,"Handling Marker W nmeadispl_temp %x", nmeadispl_temp);
      ptr = strtok(NULL, limiter);   // look for TAB
      if (ptr!=NULL)  
      {              // Generate NMEA sentence and copy to buffer  $IIXDR,W,xx,V,U001,
        if (nmeadispl_temp == 0x0ff) { 
          strncpy(bufi, ",W,", 3);  
          bufi+=3;
          char i = 0;
          while (('0' <= *(ptr+i)) && (*(ptr+i) <= '9')) {
            *bufi++ = *(ptr+i); 
            number++;
            if (i++ > 5) break;
          }
          strcpy(bufi, ",W,U1");    //  Now mA  and U001 as index 
          bufi+=5;
          number += 8;
        }
        else
        {
          strncpy(bufi, nmeastringw.nmeastring, 6);
          bufi+=6;
          *bufi++ = ',';
          number+=7;
          char i = 0;
          j = 0;
          // Value from Vitronic is in W, 
          while ( (('0' <= *(ptr+i)) && (*(ptr+i) <= '9')) || *(ptr+i)== '-') {  // How many digits ?
            i++;
          }
          DEBUG(9,"Handling Solar Power W: %i digits", i);  // i contains amount of numbers
          if (*(ptr)== '-') {*bufi++ = '-'; number++; j++;}
          if (i == 2) {*bufi++ = *(ptr+j); j++;} else {*bufi++ = '0';} // 10W
          number++;
          *bufi++ = *(ptr+j);    // W
          number++;
          *bufi++ = '.'; 
          number++;
          *bufi++ = '0'; 
          number++;

          *bufi++ = ',';
          number++;
          *bufi++ = nmeastringw.unit;
          number++;
         }
      }
    }
    else if ((marker == 'E') && (nmeadispl_temp & 0x40) || (marker == 'O') && (nmeadispl_temp & 0x20))    // handle  "H19" for Energy total 
    {
      DEBUG(9,"Handling Marker E nmeadispl_temp %x", nmeadispl_temp);
      ptr = strtok(NULL, limiter);   // look for TAB
      if (ptr!=NULL)  
      {              // Generate NMEA sentence and copy to buffer  $IIXDR,W,xx,V,U001,
        if (nmeadispl_temp == 0x0ff) { 
          if (marker == 'E') strncpy(bufi, ",E,", 3);  
          else strncpy(bufi, ",O,", 3);
          bufi+=3;
          char i = 0;
          while ((('0' <= *(ptr+i)) && (*(ptr+i) <= '9') ) || (*(ptr+i)== '-')) {   //  as long as we get numbers
            *bufi++ = *(ptr+i); 
            number++;
            if (i++ > 5) break;       // Something wrong, give up
          }
          strcpy(bufi, "0,Wh,U1");    //  Now mA  and U001 as index 
          bufi+=7;
          number += 10;
        }
        else
        {
          if (marker == 'E')  strncpy(bufi, nmeastringe.nmeastring, 6);
          else  strncpy(bufi, nmeastringo.nmeastring, 6);
          bufi+=6;
          *bufi++ = ',';
          number+=7;
          char i = 0;
          j = 0;
          // Value from Vitronic is in 10 Wh, 
          i = 0;
          while ((('0' <= *(ptr+i)) && (*(ptr+i) <= '9') ) || (*(ptr+i)== '-')) {   //  as long as we get numbers
            *bufi++ = *(ptr+i); 
            number++; 
            if (i++ > 5) break;       // Something wrong, give up
          }
          strcpy(bufi, "0,Wh");    //  Now mA  and U001 as index
          bufi+=4;
          number += 7;
          DEBUG(9,"Handling Energy: %i digits", i);
         }
      }
      else
      {
        bufi -= 2;
        printf("value of PPV missing!\n");
      }
    }
    ptr = strtok(NULL, limiter);   // look for next value
  }
 //           DEBUG(9,"Victron Done");


  strncpy(bufi, "\n\r\0", 3);    //  No Checksum implemented
  number += 3;
           DEBUG(9,"Final Data %i: %s",number, buf);
     if (number < 5)  {
         for (i=0; i<100; i++)
            DEBUG(9, "Data %i %c %x",i, bufint[i], bufint[i]);
     }
  } while (number < 10);   //  Until we got some data
  return(number);

}



/*
 * Initialise a serial interface for victron interface 
 * Args: interface specification string and pointer to interface structure
 * Retuns: Pointer to (completed) interface structure
 */
struct iface *init_victron (struct iface *ifa)
{
    char *devname;
    struct if_serial *ifs;
    int baud=B19200;        /* Default for Victron Interface */
    int ret;
    int i, j;
    struct kopts *opt;
    int qsize=DEFSERIALQSIZE;

    strncpy(nmeastring0.nmeastring, "$IIXDR\0", 7);  
    nmeastringv.nmeastring[0] = 0;
    nmeastringp.nmeastring[0] = 0;
    nmeastringw.nmeastring[0] = 0;
    nmeastringi.nmeastring[0] = 0;
    nmeastringe.nmeastring[0] = 0;
    nmeastringy.nmeastring[0] = 0;
    nmeastringo.nmeastring[0] = 0;
 
    for(opt=ifa->options;opt;opt=opt->next) {
        if (!strcasecmp(opt->var,"filename")) 
            devname=opt->val;

        else if (!strcasecmp(opt->var,"nmeastring")) {   // The NMEA string used can be configured in kplex.conf, take string from there
          char select = *(opt->val);
          switch (*(opt->val)) {
            case 'I':                       // Battery Current
              for (j=0; j<6; j++) {
                nmeastringi.nmeastring[j] = *((opt->val)+j+2);
              }
              nmeastringi.nmeastring[6] = 0;
              nmeastringi.unit = *((opt->val)+9);
              break; 
            case 'P':                       // Solar Voltage (panel)
              for (j=0; j<6; j++) {
                nmeastringp.nmeastring[j] = *((opt->val)+j+2);
              }
              nmeastringp.nmeastring[6] = 0;
              nmeastringp.unit = *((opt->val)+9);
              break; 
            case 'W':                       // Solar power 
              for (j=0; j<6; j++) {
                nmeastringw.nmeastring[j] = *((opt->val)+j+2);
              }
              nmeastringw.nmeastring[6] = 0;
              nmeastringw.unit = *((opt->val)+9);
              break; 
            case 'V':                       // Battery Voltage
              for (j=0; j<6; j++) {
                nmeastringv.nmeastring[j] = *((opt->val)+j+2);
              }
              nmeastringv.nmeastring[6] = 0;
              nmeastringv.unit = *((opt->val)+9);
              break; 
            case 'E':                       // Energy total same day 
              for (j=0; j<6; j++) {
                nmeastringe.nmeastring[j] = *((opt->val)+j+2);
              }
              nmeastringe.nmeastring[6] = 0;
              nmeastringe.unit = *((opt->val)+9);
              break; 
            case 'Y':                       // Energy Yield  Yesterday 
              for (j=0; j<6; j++) {
                nmeastringy.nmeastring[j] = *((opt->val)+j+2);
              }
              nmeastringy.nmeastring[6] = 0;
              nmeastringy.unit = *((opt->val)+9);
              break; 
            case 'O':                       // Energy Yield  overall 
              for (j=0; j<6; j++) {
                nmeastringo.nmeastring[j] = *((opt->val)+j+2);
              }
              nmeastringo.nmeastring[6] = 0;
              nmeastringo.unit = *((opt->val)+9);
              break; 
            default: 
               DEBUG(9, "No Config");
          }
        }

        else if (!strcasecmp(opt->var,"baud")) {
            if (!strcmp(opt->val,"38400"))
                baud=B38400;
            else if (!strcmp(opt->val,"9600"))
                baud=B9600;
            else if (!strcmp(opt->val,"4800"))
                baud=B4800;
            else if (!strcmp(opt->val,"19200"))
                baud=B19200;
            else if (!strcmp(opt->val,"57600"))
                baud=B57600;
            else if (!strcmp(opt->val,"115200"))
                baud=B115200;
            else {
                logerr(0,"Unsupported baud rate \'%s\' in interface specification '\%s\'",opt->val,devname);
                return(NULL);
            }
        } else if (!strcasecmp(opt->var,"qsize")) {
            if (!(qsize=atoi(opt->val))) {
                logerr(0,"Invalid queue size specified: %s",opt->val);
                return(NULL);
            }
        } else  {
            logerr(0,"unknown interface option %s",opt->var);
            return(NULL);
        }
    }

    /* Allocate serial specific data storage */
    if ((ifs = malloc(sizeof(struct if_serial))) == NULL) {
        logerr(errno,"Could not allocate memory");
        return(NULL);
    }

    /* Open interface or die */
    if ((ifs->fd=ttyopen(devname,ifa->direction)) < 0) {
        return(NULL);
    }
    DEBUG(3,"%s: opened serial device %s for %s",ifa->name,devname,
            (ifa->direction==IN)?"input":(ifa->direction==OUT)?"output":
            "input/output");

    free_options(ifa->options);

    /* Set up interface or die */
    if ((ret = ttysetup(ifs->fd,&ifs->otermios,baud,0)) < 0) {
        if (ret == -1) {
            if (tcsetattr(ifs->fd,TCSANOW,&ifs->otermios) < 0) {
                logerr(errno,"Failed to reset serial line");
            }
        }
        return(NULL);
    }
    ifs->saved=1;
    ifs->slavename=NULL;

    /* Assign pointers to read, write and cleanup routines */
    ifa->read=do_read;
    ifa->readbuf=read_victron;
    ifa->write=NULL;
    ifa->cleanup=cleanup_victron;

    /* Allocate queue for outbound interfaces */
    if (ifa->direction != IN)
        if (init_q(ifa, qsize) < 0) {
            logerr(errno,"Could not create queue");
            cleanup_serial(ifa);
            return(NULL);
        }

    /* Link in serial specific data */
    ifa->info=(void *)ifs;

    if (ifa->direction == BOTH) {
        if ((ifa->next=ifdup(ifa)) == NULL) {
            logerr(0,"Interface duplication failed");
            cleanup_serial(ifa);
            return(NULL);
        }
        ifa->direction=OUT;
        ifa->pair->direction=IN;
    }
    DEBUG(9,"Nmeastring %s: nmeaunit %c", &nmeastringv.nmeastring, nmeastringv.unit);

    return(ifa);
}

