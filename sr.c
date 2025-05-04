#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 12     /* the min sequence space for SR must be at least 2*windowsize */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet */
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ )
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static bool acked[SEQSPACE];           /* array to track if a packet has been ACKed */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    windowcount++;
    acked[sendpkt.seqnum] = false;  /* mark as not ACKed */

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer for this packet if it's the first one */
    if (windowcount == 1)
      starttimer(A, RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked, window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
  int i;
  int idx;
  bool can_slide = false;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* check if new ACK or duplicate */
    if (windowcount != 0) {
      int seqfirst = buffer[windowfirst].seqnum;
      int seqlast = buffer[windowlast].seqnum;
      
      /* check if ACK is within window */
      if (((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
          ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast))) {
        
        /* find the packet in the window */
        for (i = 0; i < windowcount; i++) {
          idx = (windowfirst + i) % WINDOWSIZE;
          if (buffer[idx].seqnum == packet.acknum && !acked[packet.acknum]) {
            /* mark as ACKed */
            acked[packet.acknum] = true;
            
            if (TRACE > 0)
              printf("----A: ACK %d is not a duplicate\n", packet.acknum);
            new_ACKs++;
            
            /* check if we can slide window */
            if (idx == windowfirst) {
              can_slide = true;
            }
            break;
          }
        }
        
        /* if we can slide window */
        if (can_slide) {
          /* slide window until we find an unACKed packet */
          while (windowcount > 0 && acked[buffer[windowfirst].seqnum]) {
            windowfirst = (windowfirst + 1) % WINDOWSIZE;
            windowcount--;
          }
          
          /* restart timer if there are still packets in window */
          stoptimer(A);
          if (windowcount > 0) {
            starttimer(A, RTT);
          }
        }
      }
    }
    else
      if (TRACE > 0)
        printf ("----A: duplicate ACK received, do nothing!\n");
  }
  else
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* 重传窗口中的第一个未确认数据包 */
  int i;
  for (i = 0; i < windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    if (!acked[buffer[idx].seqnum]) {
      if (TRACE > 0)
        printf ("---A: resending packet %d\n", buffer[idx].seqnum);
      
      tolayer3(A, buffer[idx]);
      packets_resent++;
      break; /* 只重传一个数据包后退出 */
    }
  }
  
  /* 重启计时器 */
  if (windowcount > 0) {
    starttimer(A, RTT);
  }
}

/* initialization function */
void A_init(void)
{
  int i;
  
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0 */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored */
  windowcount = 0;
  
  /* initialize acked array */
  for (i = 0; i < SEQSPACE; i++)
    acked[i] = false;
}

/********* Receiver (B) variables and procedures ************/

static int B_nextseqnum;   /* the sequence number for the next packets sent by B */

/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  struct pkt ackpkt;
  int i;
  
  /* if packet is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    
    packets_received++;
    
    /* create and send ACK packet */
    ackpkt.seqnum = NOTINUSE;
    ackpkt.acknum = packet.seqnum;
    ackpkt.checksum = 0;
    for (i = 0; i < 20; i++)
      ackpkt.payload[i] = 0;
    ackpkt.checksum = ComputeChecksum(ackpkt);
    
    /* send ACK */
    tolayer3(B, ackpkt);
    
    /* deliver data to layer 5 if it's the expected packet */
    if (packet.seqnum == B_nextseqnum) {
      tolayer5(B, packet.payload);
      B_nextseqnum = (B_nextseqnum + 1) % SEQSPACE;
    }
  }
  else {
    if (TRACE > 0)
      printf("----B: packet is corrupted, send NAK!\n");
    
    /* create and send NAK packet */
    ackpkt.seqnum = NOTINUSE;
    ackpkt.acknum = B_nextseqnum ? B_nextseqnum - 1 : SEQSPACE - 1;
    ackpkt.checksum = 0;
    for (i = 0; i < 20; i++)
      ackpkt.payload[i] = 0;
    ackpkt.checksum = ComputeChecksum(ackpkt);
    
    /* send NAK */
    tolayer3(B, ackpkt);
  }
}

/* initialization function */
void B_init(void)
{
  B_nextseqnum = 0;
}

/* functions for bidirectional communication */
void B_output(struct msg message)
{
}

void B_timerinterrupt(void)
{
}