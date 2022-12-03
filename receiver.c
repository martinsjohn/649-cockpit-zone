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
#include <wiringPiI2C.h>
#include <termios.h>

#include "state.h"

#include <time.h>

/** Configure this **/
#define LOCAL_HOST "169.254.138.247" // IP of local interface
#define R_PORT 8000

#define REMOTE_HOST "169.254.67.157"
#define S_PORT 8001
/** **/


#define TX_INTERVAL_MS 50
#define STATE_SIZE sizeof(DIJOYSTATE2_t)

//Thread IDs
pthread_t send_tid;
pthread_t send_wheel_tid;
pthread_t send_throttle_tid;
pthread_t send_brake_tid;
pthread_t send_blink_tid;

//Mutexes
pthread_mutex_t wheel_lock;
pthread_mutex_t throttle_lock;
pthread_mutex_t brake_lock;
pthread_mutex_t blink_lock;

//Global values for parts
uint8_t wheelNum;
uint8_t brakeNum;
uint8_t throttleNum;
uint8_t blinkNum;

//I2C
int i2cPort;


void *send_info(void *arg){
  while(1){
    //pthread_mutex_lock(&wheel_lock);
    printf("Sending Wheel: %d \n", wheelNum);
    wiringPiI2CWrite(i2cPort, wheelNum);
    wiringPiI2CWrite(i2cPort, throttleNum);
    wiringPiI2CWrite(i2cPort, brakeNum);
    wiringPiI2CWrite(i2cPort, blinkNum);
    //pthread_mutex_unlock(&wheel_lock);
  }
}

void *send_throttle(void *arg){
  while(1){
    //pthread_mutex_lock(&throttle_lock);
    printf("Sending Throttle: %d \n", throttleNum);
    wiringPiI2CWrite(i2cPort, throttleNum);
    //pthread_mutex_unlock(&throttle_lock);
  }
}

void *send_brake(void *arg){
  while(1){
    //pthread_mutex_lock(&brake_lock);
    printf("Sending Brake: %d \n", brakeNum);
    wiringPiI2CWrite(i2cPort, brakeNum);
    //pthread_mutex_unlock(&brake_lock);
  }
}

void *send_blink(void *arg){
  while(1){
    //pthread_mutex_lock(&blink_lock);
    printf("Sending Blink: %d \n", blinkNum);
    wiringPiI2CWrite(i2cPort, blinkNum);
    //pthread_mutex_unlock(&blink_lock);
  }
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

//void sendDataI2C(void *arg)

int main() {
  //Socket info
  int sockfd;
  char buffer[STATE_SIZE + 1];

  //UART info
  //int serialport;
  //char uartData;

  //I2C info
  int stmI2CAddr = 0;

  struct termios options;

  struct sockaddr_in servaddr = { 0 };

  /*
  if((serialport = serialOpen("/dev/ttyS0", 9600)) < 0){
    perror("failed to open serial device");
    exit(EXIT_FAILURE);
  }
  */

  if((i2cPort = wiringPiI2CSetup(stmI2CAddr)) < 0){
    perror("failed to openi2c port");
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
  //tcgetattr (serialport, &options);
  //cfmakeraw(&options);
  //options.c_cflag &= ~CSIZE;
  //options.c_cflag |= CS7;
  //options.c_cflag |= PARENB;
  // if(tcsetattr (serialport,TCSADRAIN, &options) >= 0){
  //   printf("UART Configured successfully");
  // }
// 
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

  //pthread_create(&send_tid, NULL, send_force, NULL);

  //pthread_create(&send_wheel_tid, NULL, send_info, NULL);
  //pthread_create(&send_throttle_tid, NULL, send_throttle, NULL);
  //pthread_create(&send_brake_tid, NULL, send_brake, NULL);
  //pthread_create(&send_blink_tid, NULL, send_blink, NULL);

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
    
    
    //char buf[75];
    //sprintf(buf, "Wheel: %5.5i | Throt: %5.5i | Brk: %5.5i | Blnk: %5.5i \n",
    //  state.lX, state.lY, state.lRz, blinks);
    //sprintf(buf, "ZZERTYUIOPASDFGHJKL\r\n");
    //sprintf(buf, "Wheel: %d", state.lX);
    //serialPuts(serialport, buf);
    //for(int i = 0; i < strlen(buf); i++){
    //  serialPutchar(serialport, buf[i]);
    //}
    // write(serialport, buf, 20);
    // flush()
    //printf(buf);
    //serialPuts(serialport, buf);
    //serialPrintf(serialport, buf);
    // serialFlush(serialport);
    uint8_t data[] = "Hello World\r\n";
    
    //Modify wheelNum
    //pthread_mutex_lock(&wheel_lock);
    wheelNum = (((int)state.lX / 656) + 75);
    //pthread_mutex_unlock(&wheel_lock);
    
    //Modify throttleNum
    //pthread_mutex_lock(&throttle_lock);
    throttleNum = (((int)state.lY / (-656)) - 207);
    //pthread_mutex_unlock(&throttle_lock);

    //Modify brakeNum
    //pthread_mutex_lock(&brake_lock);
    brakeNum = (((int)state.lRz / (-656)) - 207);
    //pthread_mutex_unlock(&brake_lock);
    
    //blinkNum
    //pthread_mutex_lock(&blink_lock);
    blinkNum = (uint8_t)blinks;
    //pthread_mutex_unlock(&blink_lock);
    
    //ALL DATA
    char dataAll[30];
    sprintf(dataAll, "W:%d T:%d, Br:%d, Bl:%d \r\n", 
      wheelNum, throttleNum, brakeNum, blinks);
    
    printf(dataAll);

    //Data for just wheel
    char dataWheel[1];
    sprintf(dataWheel, "%d", wheelNum);


    //I2C Sending
    
    //for(int i = 0; i < strlen((char*)dataAll); i ++){
    //  wiringPiI2CWrite(i2cPort,dataAll[i]);
    //}
    uint8_t sendData;
    sendData |= blinkNum;
    sendData = sendData << 6;
    sendData |= (throttleNum & 0x3F);
    //sendData |= blinkNum;
    

    wiringPiI2CWrite(i2cPort, sendData);
    //wiringPiI2CWrite(i2cPort, throttleNum);
    //wiringPiI2CWrite(i2cPort, brakeNum);
    //wiringPiI2CWrite(i2cPort, blinkNum);

  }

  return 0;
}


