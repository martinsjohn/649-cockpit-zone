#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include "pthread.h"
#include <inttypes.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <termios.h>

#include "state.h"

#include <time.h>

/** Configure this **/
#define LOCAL_HOST "169.254.138.247" // IP of local interface
#define R_PORT 8000

#define REMOTE_HOST "169.254.67.157"
#define S_PORT 8001
/** **/


#define TX_INTERVAL_MS 1000
#define STATE_SIZE sizeof(DIJOYSTATE2_t)

void delay_sec(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;
 
    // Storing start time
    clock_t start_time = clock();
 
    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}

void *send_force(void *arg) {
  int8_t force = 0;
  int sockfd;
  struct sockaddr_in servaddr;

  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(S_PORT);
  servaddr.sin_addr.s_addr = inet_addr(REMOTE_HOST);
  
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("failed to create socket");
    exit(EXIT_FAILURE);
  }

  while (1) {
    usleep(TX_INTERVAL_MS * 2000);
    printf("Send force %d\n", force);
    force += 5;
    sendto(sockfd, (char*) &force, 1, MSG_CONFIRM,
		    (struct sockaddr *) &servaddr, sizeof(servaddr));
    
  }
}

int main() {
  //Socket info
  int sockfd;
  char buffer[STATE_SIZE + 1];

  //UART info
  int serialport;
  char uartData;

  struct termios options;

  struct sockaddr_in servaddr = { 0 };

  if((serialport = serialOpen("/dev/ttyS0", 115200)) < 0){
    perror("failed to open serial device");
    exit(EXIT_FAILURE);
  }

  if(wiringPiSetup() == -1){
    perror("unable to start wiringPi");
    exit(EXIT_FAILURE);
  }


  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("failed to create socket");
    exit(EXIT_FAILURE);
  }
  // Should try configuring STM Uart reciver before messing
  // with these much more confusing settings
  //uart advanced settings
  tcgetattr (serialport, &options);
  cfmakeraw(&options);
  //options.c_cflag &= ~CSIZE;
  //options.c_cflag |= CS7;
  //options.c_cflag |= PARENB;
  if(tcsetattr (serialport,TCSANOW, &options) >= 0){
    printf("UART Configured successfully");
  }

  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(R_PORT);
  servaddr.sin_addr.s_addr = inet_addr(LOCAL_HOST);

  if (bind(sockfd, (const struct sockaddr *) &servaddr, 
            sizeof(servaddr)) < 0) {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  DIJOYSTATE2_t state;
  char recvbuf[sizeof(DIJOYSTATE2_t) + 8]; //4-counter, 4-blinks
  pthread_t send_tid;
  //pthread_create(&send_tid, NULL, send_force, NULL);

  while(1) {
    //printf("Wait recv\n");
    int n, len;
    n = recvfrom(sockfd, recvbuf, STATE_SIZE, MSG_WAITALL,
                  (struct sockaddr *) &servaddr, &len);
    uint32_t packet_ct = ((uint32_t*) recvbuf)[1];
    uint32_t blinks = ((uint32_t*) recvbuf)[0];
    memcpy(&state, recvbuf + 8, sizeof(state));
    printf("Receive state (Pkt: %8X) :  Wheel: %d | Throttle: %d | Brake: %d\n",
      packet_ct, state.lX, state.lY, state.lRz);
    //char msg[20] = "Howdy";
    
    
    uint8_t buf[75];
    //sprintf(buf, "Wheel: %5.5i | Throt: %5.5i | Brk: %5.5i | Blnk: %5.5i \n",
    //  state.lX, state.lY, state.lRz, blinks);
    sprintf(buf, "QWERTYUIOPASDFGHJKL");

    for(int i = 0; i < strlen(buf); i++){
        serialPutchar(serialport, buf[i]);
    }
    printf(buf);
    //serialPuts(serialport, buf);
    //serialPrintf(serialport, buf);
    //delay_sec(0.5);
    serialFlush(serialport);

  }

  return 0;
}


