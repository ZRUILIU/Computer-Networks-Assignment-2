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
  int i;
  int idx;
  
  if (TRACE > 0)
    printf("----A: time out, resend unACKed packets!\n");

  /* In SR, we only resend unACKed packets */
  for (i = 0; i < windowcount; i++) {
    idx = (windowfirst + i) % WINDOWSIZE;
    if (!acked[buffer[idx].seqnum]) {
      if (TRACE > 0)
        printf ("---A: resending packet %d\n", buffer[idx].seqnum);
      
      tolayer3(A, buffer[idx]);
      packets_resent++;
    }
  }
  
  /* restart timer */
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

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt rcvbuffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static bool received[SEQSPACE]; /* array to track received packets */
static int rcv_base; /* base of the receive window */

/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  struct pkt ackpkt;
  struct pkt nakpkt;
  int i;
  int rcv_end;
  bool in_window;
  
  /* if packet is not corrupted */
  if (!IsCorrupted(packet)) {
    /* check if packet is within receive window */
    rcv_end = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;
    in_window = false;
    
    if ((rcv_base <= rcv_end && packet.seqnum >= rcv_base && packet.seqnum <= rcv_end) ||
        (rcv_base > rcv_end && (packet.seqnum >= rcv_base || packet.seqnum <= rcv_end))) {
      in_window = true;
    }
    
    if (in_window) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      
      packets_received++;
      received[packet.seqnum] = true;
      
      /* store packet in buffer */
      rcvbuffer[(packet.seqnum - rcv_base + SEQSPACE) % SEQSPACE] = packet;
      
      /* create and send ACK packet */
      ackpkt.seqnum = NOTINUSE;
      ackpkt.acknum = packet.seqnum;
      ackpkt.checksum = 0;
      for (i = 0; i < 20; i++)
        ackpkt.payload[i] = 0;
      ackpkt.checksum = ComputeChecksum(ackpkt);
      
      /* send ACK */
      tolayer3(B, ackpkt);
      
      /* deliver in-order packets to layer 5 */
      while (received[rcv_base]) {
        /* deliver to layer 5 */
        tolayer5(B, rcvbuffer[(rcv_base - rcv_base + SEQSPACE) % SEQSPACE].payload);
        
        /* advance receive window */
        received[rcv_base] = false;
        rcv_base = (rcv_base + 1) % SEQSPACE;
      }
    }
    else {
      /* packet outside window, send ACK anyway */
      if (TRACE > 0)
        printf("----B: packet %d outside receive window, send ACK!\n", packet.seqnum);
      
      ackpkt.seqnum = NOTINUSE;
      ackpkt.acknum = packet.seqnum;
      ackpkt.checksum = 0;
      for (i = 0; i < 20; i++)
        ackpkt.payload[i] = 0;
      ackpkt.checksum = ComputeChecksum(ackpkt);
      
      /* send ACK */
      tolayer3(B, ackpkt);
    }
  }
  else {
    if (TRACE > 0)
      printf("----B: packet is corrupted, send NAK!\n");
    
    /* create and send NAK packet */
    nakpkt.seqnum = NOTINUSE;
    nakpkt.acknum = rcv_base ? rcv_base - 1 : SEQSPACE - 1;
    nakpkt.checksum = 0;
    for (i = 0; i < 20; i++)
      nakpkt.payload[i] = 0;
    nakpkt.checksum = ComputeChecksum(nakpkt);
    
    /* send NAK */
    tolayer3(B, nakpkt);
  }
}

/* initialization function */
void B_init(void)
{
  int i;
  
  expectedseqnum = 0;
  B_nextseqnum = 1;
  rcv_base = 0;
  
  /* initialize received array */
  for (i = 0; i < SEQSPACE; i++) {
    received[i] = false;
  }
}

/* functions for bidirectional communication */
void B_output(struct msg message)
{
}

void B_timerinterrupt(void)
{
}