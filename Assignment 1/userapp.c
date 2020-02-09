#include <stdio.h> 
#include<stdlib.h>
#include<fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <stdint.h>
#include<sys/ioctl.h>
//#include "chardev.h"

#define MAGIC_NUM 'Z'

#define CHANNEL 1
#define ALLIGNMENT 2

#define SET_CHANNEL _IOW(MAGIC_NUM, CHANNEL, unsigned int)
#define SET_ALLIGNMENT _IOW(MAGIC_NUM, ALLIGNMENT, unsigned int)

int main()

{ 
  int fd;
  uint16_t val;
  //uint16_t flag=1;
  uint32_t chnl, align;

  //fd=open(adc, O_RDONLY);
  fd = open("/dev/adc8", O_RDWR);
  
  if(fd <0)
  {
  
    printf("cannot read source file adc8\n");
    return 0;
  }
  printf("Enter the Channel number\n");
        scanf("%d",&chnl);
        
        
        printf("Writing Value to Driver\n");
        ioctl(fd, SET_CHANNEL, (int32_t*) &chnl);
        
      

        printf("Enter the alignment : 0 for lower bits, 1 for higher bits\n");
        scanf("%d",&align);
        printf("Writing Value to Driver\n");
        ioctl(fd, SET_ALLIGNMENT, (int32_t*) &align);

  read(fd,&val,2);
  printf("adc value: %d\n",val);

  close(fd);
  
  
  return 0;
}