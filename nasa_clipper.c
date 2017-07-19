/* nasa_clipper.c
 * This file is part of kplex
 * Copyright Keith Young 2012 - 2016
 * For copying information see the file COPYING distributed with this software
 *
 * This file contains code for Nasa Clipper like . $SDDPT,1.8,0.0 
   It is intended for the Raspberry PI 2 and 3 and uses the I2C slave interface located on GPIO 18 and 19 )Pins 12 and xx on the GPIO. 
   Pin 19 is not available on the PI 1 so this cannot be used. 
   The NASA Clipper has an output to connect to a slave device which outputs the data for the display. 

    
    At the back of your NASA Clipper should be a round connector
    with 5 pins, looking roughly like this ASCII art:
      Pin Location          Pin Number
          |                     2
      \       /              4     5
    -           -          1         3
          
          O                     6
          
  Pin 1: SCL  -> Connect to GPIO 19  -> Pin 35
  Pin 2: GND  
  Pin 3: SDA  -> Connect to GPIO 18  -> Pin 12
  Pin 4: 12V
  Pin 5: GND

  For Detail refer also to:
  
  http://wiki.openseamap.org/wiki/De:NASA_Clipper_Range
  


 * The BSC peripheral uses GPIO 18 (SDA) and 19 (SCL) in I2C mode
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
#include <pigpio.h>  // Low level SPI driver library

#define HIGH 1
#define LOW 0

/*
 * Cleanup interface on exit
 * Args: pointer to interface
 * Returns: Nothing
 */
void cleanup_nasa_clipper(iface_t *ifa)
{


  // Need to fill in later
}

  bsc_xfer_t xfer;

// The NASA Clipper sends a 12 byte I2C data packet, this data is directly send to a pcf8566p
// LCD Driver. The first byte is the address and the write direction, the next eleven bytes is data.
// The first 5 bytes is a command, the proceeding 6 bytes are data
// positions and contain the single LCD elements. 
// Example data {0x7c,0xce,0x80,0xe0,0xf8,0x70,0x00,0x00,0x00,0x00,0x00,0x00};
//               addr   0    1    2    3    4    5    6    7    8    9    10
//                      com  com  com com   com dta  dta  dta  dta  dta  dta
// Example depth  : 23.3
// Digit number   : 12.3


char I2C_predata[5] =   {0xce,0x80,0xe0,0xf8,0x70};
char depth_mask[6] =    {0x01,0,0,0,0,0};
char decpoint_mask[6] = {0,0,0,0x80,0x0,0x0};
char metres_mask[6] =   {0,0,0,0x40,0x0,0x0};
char digit3_mask[6] =   {0,0xbf,0,0,0,0};
char digit3[10][6] = {                   // from https://en.wikipedia.org/wiki/Seven-segment_display
                   { 0, 0xbb,0,0,0,0  }, // zero, a,b,c,d,e,f,/g
                   { 0, 0x11, 0,0,0,0 }, // one /a,b,c,/d,/e,/f,/g
                   { 0, 0x9e, 0,0,0,0 }, // two a,b,/c,d,e,/f,g
                   { 0, 0x97, 0,0,0,0 }, // three a,b,c,d,/e,/f,g
                   { 0, 0x35, 0,0,0,0 }, // four /a,b,c,/d,/e,f,g
                   { 0, 0xa7, 0,0,0,0 }, // five a,/b,c,d,/e,f,g
                   { 0, 0xaf, 0,0,0,0 }, // six a,/b,c,d,e,f,g
                   { 0, 0x91, 0,0,0,0 }, // seven a,b,c,/d,/e,/f,/g
                   { 0, 0xbf, 0,0,0,0 }, // eight a,b,c,d,e,f,g
                   { 0, 0xb7, 0,0,0,0 }, // nine a,b,c,d,/e,f,g
                 };
                 
char digit2_mask[6] = {0xfe,0,0,0,0,0};
char digit2[10][6] = {                   // from https://en.wikipedia.org/wiki/Seven-segment_display
                   { 0xee, 0, 0,0,0,0  },// zero, a,b,c,d,e,f,/g
                   { 0x44, 0, 0,0,0,0 }, // one /a,b,c,/d,/e,/f,/g
                   { 0xb6, 0, 0,0,0,0 }, // two a,b,/c,d,e,/f,g
                   { 0xd6, 0, 0,0,0,0 }, // three a,b,c,d,/e,/f,g
                   { 0x5c, 0, 0,0,0,0 }, // four /a,b,c,/d,/e,f,g
                   { 0xda, 0, 0,0,0,0 }, // five a,/b,c,d,/e,f,g
                   { 0xfa, 0, 0,0,0,0 }, // six a,/b,c,d,e,f,g
                   { 0x46, 0, 0,0,0,0 }, // seven a,b,c,/d,/e,/f,/g
                   { 0xfe, 0, 0,0,0,0 }, // eight a,b,c,d,e,f,g
                   { 0xde, 0, 0,0,0,0 }, // nine a,b,c,d,/e,f,g
                 };
                 

char digit1_mask[6] = {0,0,0,0,0x2f,0xc0};
char digit1[10][6] = {                      // from https://en.wikipedia.org/wiki/Seven-segment_display
                   { 0, 0, 0,0,0x2e,0xc0 }, // zero, a,b,c,d,e,f,/g
                   { 0, 0, 0,0,0x04,0x40 }, // one /a,b,c,/d,/e,/f,/g
                   { 0, 0, 0,0,0x27,0x80 }, // two a,b,/c,d,e,/f,g
                   { 0, 0, 0,0,0x25,0xC0 }, // three a,b,c,d,/e,/f,g
                   { 0, 0, 0,0,0x0d,0x40 }, // four /a,b,c,/d,/e,f,g
                   { 0, 0, 0,0,0x29,0xC0 }, // five a,/b,c,d,/e,f,g
                   { 0, 0, 0,0,0x2b,0xC0 }, // six a,/b,c,d,e,f,g
                   { 0, 0, 0,0,0x24,0x40 }, // seven a,b,c,/d,/e,/f,/g
                   { 0, 0, 0,0,0x2f,0xc0 }, // eight a,b,c,d,e,f,g
                   { 0, 0, 0,0,0x2d,0xc0 }, // nine a,b,c,d,/e,f,g
                 };


char nasa_rcnt = 0;
char nasa_buf[13]; 
char nasa_rec_state = 0;
/*
 * Read from a serial interface from Victron Controller, comvert to NMEA end put into Buffer
 * Args: pointer to interface structure pointer to buffer
 * Returns: Number of bytes read, zero on error or end of file
 */
ssize_t read_nasa_clipper(struct iface *ifa, char *NMEADPTstr)
{
  struct if_serial *ifs = (struct if_serial *) ifa->info;   // Get serial specific information from iface struct. Used to get file descriptor
  char* ptr=NULL;
  char* bufi;
#define IBUFSIZE 400
  unsigned char bufint[IBUFSIZE];     // Buffer for data 
int I2C_nrec;   // Number of received bytes from I2C
char data_handler[11];
char data[11];
char flag_valid_data = 0;
  unsigned char sum = 1;


  int i,j;
  char digit_tmp0,digit_tmp1,dig1,dig2,dig3,dec_point,checksum;
  char dptstr[5],cksstr[3],ind_dptstr;
  static char incomingByte,command[60], DEBUG_MODE=0;
  char flag_depth;

  int status;
  nasa_rec_state = 0;

  while (nasa_rec_state < 12) {
    status = bscXfer(&xfer);    // Get data
    I2C_nrec = xfer.rxCnt;   // 

   j = 0;
   while (xfer.rxCnt != 0) {   // While there is something, copy it and increase counters
    //DEBUG(8,"RxCnt %i state %i j %i", xfer.rxCnt, nasa_rec_state, j);
    if (nasa_rec_state > 4) {       // Data Reception ongoing
      nasa_buf[nasa_rec_state] = xfer.rxBuf[j];  
      nasa_rec_state++; 
      if (nasa_rec_state > 10) break; 
    }
    else if ((xfer.rxBuf[j] == 0xCE) && (nasa_rec_state == 0)){ nasa_rec_state = 1; nasa_buf[0] = 0xCE; }
    else if ((xfer.rxBuf[j] == 0x80) && (nasa_rec_state == 1)){ nasa_rec_state = 2; nasa_buf[1] = 0x80; } 
    else if ((xfer.rxBuf[j] == 0xE0) && (nasa_rec_state == 2)){ nasa_rec_state = 3; nasa_buf[2] = 0xE0; }
    else if ((xfer.rxBuf[j] == 0xF8) && (nasa_rec_state == 3)){ nasa_rec_state = 4; nasa_buf[3] = 0xF8; }
    else if ((xfer.rxBuf[j] == 0x70) && (nasa_rec_state == 4)){ nasa_rec_state = 5; nasa_buf[4] = 0x70; }
    else { 
       nasa_rec_state = 0; 
    }
    xfer.rxCnt--;
    j++; 
   }
  }

  if (nasa_rec_state > 10)  // Got 11 Bytes, this looks good
  {
  //DEBUG(8,"Got NASA" );
            //DEBUG(8,"Processing" );
    ind_dptstr = 0;
    
    // Copy the rawdata
    for(j=0;j<11;j++)
      {
        data[j] = nasa_buf[j];
            //DEBUG(8,"Data %i  %x", j, data[j]);
      }


    // Check if the first 5 byte (the command) are correct
    // They seem to stay always the same
    for(i=0;i<5;i++)
    {
      dptstr[i] = 0;
      
      if( (data[i] & 0xFF ) == (I2C_predata[i] & 0xFF) )
      {
        flag_valid_data = HIGH;
      }
      else
      {
        flag_valid_data = LOW;     // Wrong data, 
        break;
      }
    }
    if (flag_valid_data)
    {
            //DEBUG(8,"Data OK");
      // Decode the digits
      dig1 = 'N';
      dig2 = 'N';
      dig3 = 'N';
      dec_point = 'N';
  
       // DIGIT 3 
      digit_tmp0 = data[6] & digit3_mask[1];
      for(i=0;i<10;i++)
      {
        if((digit3[i][1] & 0xFF) == (digit_tmp0 & 0xFF))
        {
          dig3 = '0' + i;
          break;
        }
      }
      // decimal point
      if((data[8] & decpoint_mask[3] & 0xFF) == (0x80))
      {
        dec_point = '.';
      }
      
      // We only consider data good, when the "DEPTH" symbol appears on the LCD
      if((data[5] & depth_mask[0] & 0xFF) == (0x01))
      {
        flag_depth = HIGH;
      }
      else
      {
        flag_depth = LOW;
      }
      
      
      // DIGIT 2 
      digit_tmp0 = data[5] & digit2_mask[0];
      for(i=0;i<10;i++)
      {
        if((digit2[i][0] & 0xFF) == (digit_tmp0 & 0xFF))
        {        
          dig2 = '0' + i;
          break;
        }
      }
     // DIGIT 1
      digit_tmp0 = data[9] & digit1_mask[4];
      digit_tmp1 = data[10] & digit1_mask[5];
      for(i=0;i<10;i++)
      {
        if(((digit1[i][4] & 0xFF) == (digit_tmp0 & 0xFF)) &
        ( (digit1[i][5] & 0xFF) == (digit_tmp1 & 0xFF)  ))
        {
          dig1 = '0' + i;
          break;
        }
      }
      
      i = 0;
      // Do we have good data? (flag_depth and at least one digit
      if(((dig1 != 'N') | (dig2 != 'N') | (dig3 != 'N')) & (flag_depth == HIGH))
        {
          ind_dptstr = 0;
          if(dig1 != 'N')
          {
            dptstr[ind_dptstr] = dig1;
            ind_dptstr ++;
          }
          if(dig2 != 'N')
          {
            dptstr[ind_dptstr] = dig2;
            ind_dptstr ++;
          }
          if(dec_point != 'N')
          {
            dptstr[ind_dptstr] = dec_point;
            ind_dptstr ++;
          }
          if(dig3 != 'N')
          {
            dptstr[ind_dptstr] = dig3;
            ind_dptstr ++;
          }
          dptstr[ind_dptstr] = '\0';
          strcpy(NMEADPTstr,"$SDDPT,"); 
          strcat(NMEADPTstr,dptstr);
          strcat(NMEADPTstr,",0.0*");
          // Calculate Checksum
          checksum = 0;
          i=0;
         while(1)
          {
            i++;
            if(NMEADPTstr[i] == '*')
              break;
            checksum = checksum ^ NMEADPTstr[i];
          }
          
          sprintf(cksstr,"%X",checksum & 0xFF);
          strcat(NMEADPTstr,cksstr);
          strcat(NMEADPTstr,"\r\n");
          if(flag_depth == HIGH)
          {
            DEBUG(8, "%s", NMEADPTstr);
          }
        }
      else
        {
            DEBUG(8, "Bad Data from Nasa Clipper");
        }
    }
    // Get rid of old data
    for(i=0;i<11;i++)
    {
       data[i] = 0;
    }
  }

  return(19);

}



/*
 * Initialise  
 * Args: interface specification string and pointer to interface structure
 * Retuns: Pointer to (completed) interface structure
 */
struct iface *init_nasa_clipper (struct iface *ifa)
{
    char *devname;
    struct if_serial *ifs;
    int baud=B19200;        /* Default for Victron Interface */
    int ret;
    struct kopts *opt;
    int qsize=1;
    
    for(opt=ifa->options;opt;opt=opt->next) {
        if (!strcasecmp(opt->var,"filename"))
            devname=opt->val;
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

if (gpioInitialise() < 0)
{
        logerr(errno,"Could not initilize SPI driver");
            DEBUG(8,"Init pigpio failed");
   // pigpio initialisation failed.
}
else
{
            DEBUG(8,"Init pigpio ok");
       // pigpio initialised okay.
}

gpioSetMode(18, PI_INPUT);  // Set GPIO17 as input.
gpioSetMode(19, PI_INPUT);  // Set GPIO17 as input.

gpioSetMode(18, PI_ALT3);  // set GPIO17 to ALT3
gpioSetMode(19, PI_ALT3);  // set GPIO17 to ALT3 / I2C Input

xfer.control = (0x3E<<16) | 0x205;  // I2C Address 0x3e, enable I2C and enable, enable receive, enable transmit
xfer.txCnt = 0;
int  status = bscXfer(&xfer);
            DEBUG(8,"Status Pin 18 %x Pin 19 %x %x", gpioGetMode(18), gpioGetMode(19), status);


    /* Assign pointers to read, write and cleanup routines */
    ifa->read=do_read;
    ifa->readbuf=read_nasa_clipper;
    ifa->write=NULL;
    ifa->cleanup=NULL;          // cleanup_victron;


    /* Link in nasa specific data */
    ifa->info=(void *)ifs;

    if (ifa->direction == BOTH) {
        if ((ifa->next=ifdup(ifa)) == NULL) {
            logerr(0,"Interface duplication failed");
            cleanup_serial(ifa);
            return(NULL);
        }
        ifa->direction=IN;
        ifa->pair->direction=IN;
    }
    return(ifa);
}

