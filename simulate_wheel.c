/* 
1st thread is listening to the incoming messages of force and just printing them
2nd thread is send the state
3rd thread, it takes input from the terminal and update the states
  you can change the step size by which you want to change the wheel, 
  throttle and brake from the terminal.
*/


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

#include "state.h"

/** Configure this **/
#define LOCAL_HOST "169.254.138.247" // IP of local interface
#define R_PORT 8000

#define REMOTE_HOST "172.26.173.84" //Reciever IP
#define S_PORT 8001
/** **/


#define TX_INTERVAL_MS 1000
#define STATE_SIZE sizeof(DIJOYSTATE2_t)
//global state and mutex
DIJOYSTATE2_t state;
pthread_mutex_t lock;

void *send_state(void *arg) {
  int32_t counter = 0;
  char buffer_to_send[STATE_SIZE+4];
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
    usleep(TX_INTERVAL_MS * 1000);
    pthread_mutex_lock(&lock);
    counter += 1;
    memcpy(buffer_to_send, &counter, 4);
    memcpy(buffer_to_send+4, &state, sizeof(state));
    sendto(sockfd, (char*) &buffer_to_send, STATE_SIZE+4, MSG_CONFIRM,
		    (struct sockaddr *) &servaddr, sizeof(servaddr));
    pthread_mutex_unlock(&lock);
  }
}


void *get_input(void *arg)
{
    int wheel_temp,throttle_temp,brake_temp;    
    state.lX=0;
    state.lY=0;
    state.lRz=0;

    while (1){
        printf("Wheel: ");
        scanf("%d",&wheel_temp);
        printf("Throttle: ");
        scanf("%d", &throttle_temp);
        printf("brake: ");
        scanf("%d", &brake_temp);
   
        pthread_mutex_lock(&lock);
        state.lX +=wheel_temp;
        state.lY +=throttle_temp;
        state.lRz +=brake_temp;
        pthread_mutex_unlock(&lock);
        }
 
}


int main() {
  int sockfd;
  struct sockaddr_in servaddr = { 0 };

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("failed to create socket");
    exit(EXIT_FAILURE);
  }

  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(R_PORT);
  servaddr.sin_addr.s_addr = inet_addr(LOCAL_HOST);

  if (bind(sockfd, (const struct sockaddr *) &servaddr, 
            sizeof(servaddr)) < 0) {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  char recvbuf[1];
  pthread_t send_tid;
  pthread_t get_tid;
  pthread_create(&send_tid, NULL, send_state, NULL);
  pthread_create(&get_tid, NULL, get_input, NULL);

  while(1) {
   printf("Wait recv\n");
    int n, len;
   n = recvfrom(sockfd, recvbuf, 1, MSG_WAITALL,
                  (struct sockaddr *) &servaddr, &len);
    uint8_t force = ((uint8_t*) recvbuf)[0];
    printf("message recieved to update the force  to %d", force);
  }

  return 0;
}
