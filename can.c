#include "driver.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"

#if CAN_ENABLE

#include "can.h"
#include <stdio.h>
#include "kinetis_can.h"

#define _bus CAN2
#define nvicIrq IRQ_CAN2
#define SIZE_LISTENERS 4
#define SIZE_RX_SYNC_BUFFER 4
#define SIZE_TX_BUFFER 8

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

typedef struct can_tx_buffer_t{
    volatile uint_fast16_t head;
    volatile uint_fast16_t tail;
    uint_fast16_t size;
    volatile uint_fast16_t count;
    CAN_message_t data[SIZE_TX_BUFFER];
} can_tx_buffer_t;

typedef struct can_rx_sync_buffer_t{
    volatile uint_fast16_t head;
    // volatile uint_fast16_t tail;
    uint_fast16_t size;
    volatile uint_fast16_t count;
    CAN_message_t data[SIZE_RX_SYNC_BUFFER];
} can_rx_sync_buffer_t;

static CAN_sync_message_t sync_msg = {0};

static can_rx_sync_buffer_t rx_sync_buf = {0}; 
static can_tx_buffer_t tx_buf = {0};

static volatile bool spin_lock = false;
static CANListener *listener[SIZE_LISTENERS];

static int mailboxOffset();
void canbus_execute_buf(sys_state_t state);
CAN_sync_message_t* canbus_write_sync_msg(CAN_message_t *msg, bool enable);
volatile uint32_t fifo_filter_table[32][6] = {0};
volatile uint32_t mb_filter_table[64][6] = {0};
static _MB_ptr _mbHandlers[64] = {0}; /* individual mailbox handlers */
static _MB_ptr _mainHandler = 0; /* global mailbox handler */
static _MB_ptr _mbTxHandlers[64] = {0}; /* individual mailbox tx handlers */
static _MB_ptr _mainTxHandler = {0}; /* global mailbox handler */
static uint32_t currentBitrate = 0;
static uint8_t mailbox_reader_increment = 0;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;
static bool initOK = false;
static bool canbus_On = false;

static void report_Frame(CAN_message_t *frame){
	char msgString[80];
	sprintf(msgString, "Frame mb:%u id:%02lu len:%01u %02x %02x %02x %02x %02x %02x %02x %02x",
						frame->mb, frame->id, frame->len,
						frame->buf[0],frame->buf[1],frame->buf[2],frame->buf[3],
						frame->buf[4],frame->buf[5],frame->buf[6],frame->buf[7]
						);
	report_message(msgString,-1);
  char code[16];
  switch (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, frame->mb)))
  {
  case 0: sprintf(code,"RX_INACTIVE");      break;
  case 1: sprintf(code,"RX_BUSY");          break;
  case 2: sprintf(code,"RX_FULL");          break;
  case 4: sprintf(code,"RX_EMPTY");         break;
  case 6: sprintf(code,"RX_OVERRUN");       break;
  case 8: sprintf(code,"TX_INACTIVE");      break;
  case 9: sprintf(code,"TX_ABORT");         break;
  case 10: sprintf(code,"TX_RESPONSE");      break;
  case 12: sprintf(code,"TX_ONCE");          break;
  case 14: sprintf(code,"TX_RESPONSE_TEMPO");break;
  default:    break;
  }
  
  uint16_t time = frame->timestamp;
  float pos = 0.0f, cpr = 0.0f;
  memcpy(&pos, &frame->low, sizeof(float));
  memcpy(&cpr, &frame->high, sizeof(float));
  sprintf(msgString, "code:%s time:%u cpr:%4.0f pos:%.0f",code,time,cpr,pos);
	report_message(msgString,-1);

}

static bool add_sync_msg(can_rx_sync_buffer_t *buf, CAN_message_t *msg)
{
  // NVIC_DISABLE_IRQ(IRQ_CAN2);
  uint_fast16_t nextEntry;
  bool result = true;
  nextEntry = (buf->head + 1) % buf->size;
  /* check if the ring buffer is full */
  if (nextEntry == buf->size) {
    canbus_write_sync_msg(0, false);
    sync_msg.ready = true;
    // protocol_enqueue_rt_command(canbus_execute_buf);
    // system_set_exec_state_flag(EXEC_RT_COMMAND);
  }
  else{
    /* add the element to the ring */
    if (sync_msg.msg)
      // sync_msg.msg_prev = sync_msg.msg;
    *sync_msg.msg = buf->data[buf->head];
    memcpy (sync_msg.msg, msg, sizeof(CAN_message_t ));
    /* bump the head to point to the next free entry */
    report_Frame(&buf->data[buf->head]);
    buf->head = nextEntry;
    buf->count++;
    // FLEXCANb_IFLAG1(_bus) |= (1UL << sync_msg.mb);
  }
  // NVIC_ENABLE_IRQ(IRQ_CAN2);
  return result;
}

static void init_buf (can_tx_buffer_t *buf, uint32_t size){
	buf->head = 0;
	buf->tail = 0;
  buf->size = size;
  buf->count = 0;
  for (int i = 0; i < SIZE_TX_BUFFER; i++){ buf->data[i] = (CAN_message_t){0}; }
  // memset(&buf->data,0,sizeof(buf->data));
}

static void init_sync_buf (can_rx_sync_buffer_t *buf, uint32_t size){
	buf->head = 0;
	// buf->tail = 0;
  buf->size = size;
  buf->count = 0;
  // buf->next = NULL;
  // buf->data = NULL;
  for (int i = 0; i < size; i++){ buf->data[i] = (CAN_message_t){0}; }
  // memset(&buf->data,0,sizeof(buf->data));
}

static bool add_buf (can_tx_buffer_t *buf, CAN_message_t *msg)
{
  uint16_t nextEntry;
  bool result = true;
  nextEntry = (buf->head + 1) % buf->size;
  /* check if the ring buffer is full */
  while (spin_lock);
  if (nextEntry == buf->tail) {
    protocol_enqueue_rt_command(canbus_execute_buf);
    system_set_exec_state_flag(EXEC_RT_COMMAND);
    // while (spin_lock);
  }
  else{
    /* add the element to the ring */
    memcpy (&buf->data[buf->head], msg, sizeof (CAN_message_t));
    /* bump the head to point to the next free entry */
    buf->head = nextEntry;
    buf->count++;
  }
  return result;
}

static bool remove_buf (can_tx_buffer_t *buf, CAN_message_t *msg)
{
  bool result = true;
  /* check if the ring buffer has data available */
  if (buf->head == buf->tail) {
    result = false;
  }
  else{
    /* copy the message */
    memcpy (msg, &buf->data[buf->tail], sizeof (CAN_message_t));
    /* bump the tail pointer */
    buf->tail = (buf->tail + 1) % buf->size;
    buf->count--;
  }
  return result;
}

static void FLEXCAN_ExitFreezeMode() {
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_HALT;
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
}

static void FLEXCAN_EnterFreezeMode() {
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT;
  while (!(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK));
}

static uint64_t readIFLAG() {
  return (((uint64_t)FLEXCANb_IFLAG2(_bus) << 32) | FLEXCANb_IFLAG1(_bus));
}

static void writeIFLAG(uint64_t value) {
  FLEXCANb_IFLAG2(_bus) = value >> 32;
}

static void writeIFLAGBit(int8_t mb_num) {
  if ( mb_num < 32 ) FLEXCANb_IFLAG1(_bus) |= (1UL << mb_num);
  else FLEXCANb_IFLAG2(_bus) |= (1UL << (mb_num - 32));
}

static void writeIMASK(uint64_t value) {
  FLEXCANb_IMASK2(_bus) = value >> 32;
}

static uint64_t readIMASK() {
  return (((uint64_t)FLEXCANb_IMASK2(_bus) << 32) | FLEXCANb_IMASK1(_bus));
}

static void writeIMASKBit(int8_t mb_num, bool set) {
  if ( mb_num < 32 ){ 
    if (set) (FLEXCANb_IMASK1(_bus)) |= (1UL << mb_num); else FLEXCANb_IMASK1(_bus) &= ~(1UL << mb_num);
  }
  else{
    if(set) FLEXCANb_IMASK2(_bus) |= (1UL << (mb_num - 32)); else FLEXCANb_IMASK2(_bus) &= ~(1UL << (mb_num - 32));
  }
}

static int getFirstTxBox() {
  for (int8_t i = mailboxOffset() + 1; i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
    // if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) >> 3) ) return i; // if TX
    if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) == FLEXCAN_MB_CODE_TX_INACTIVE ) return i;
  }
  return -1;
}

static uint32_t getClock() {
  uint8_t clocksrc[4] = {60, 24, 80, 0};
  return clocksrc[(CCM_CSCMR2 & 0x300) >> 8];
}

static void setBaudRate(uint32_t baud, int listen_only) {
  currentBitrate = baud;

  uint32_t clockFreq = getClock() * 1000000;

  uint32_t divisor = 0, bestDivisor = 0, result = clockFreq / baud / (divisor + 1);
  int error = baud - (clockFreq / (result * (divisor + 1))), bestError = error;

  bool frz_flag_negate = !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
  FLEXCAN_EnterFreezeMode();

  while (result > 5) {
    divisor++;
    result = clockFreq / baud / (divisor + 1);
    if (result <= 25) {
      error = baud - (clockFreq / (result * (divisor + 1)));
      if (error < 0) error *= -1;
      if (error < bestError) {
        bestError = error;
        bestDivisor = divisor;
      }
      if ((error == bestError) && (result > 11) && (result < 19)) {
        bestError = error;
        bestDivisor = divisor;
      }
    }
  }

  divisor = bestDivisor;
  result = clockFreq / baud / (divisor + 1);

  if ((result < 5) || (result > 25) || (bestError > 300)) {
    if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
    char msg[80];
    sprintf(msg,"CAN setClock Fail result:%lu div:%lu bestDiv:%lu baud:%lu\r\n",result,divisor,bestDivisor,baud);
    report_message(msg,Message_Warning);
    return;
  }

  result -= 5; // the bitTimingTable is offset by 5 since there was no reason to store bit timings for invalid numbers
  uint8_t bitTimingTable[21][3] = {
    {0, 0, 1}, //5
    {1, 0, 1}, //6
    {1, 1, 1}, //7
    {2, 1, 1}, //8
    {2, 2, 1}, //9
    {2, 3, 1}, //10
    {2, 3, 2}, //11
    {2, 4, 2}, //12
    {2, 5, 2}, //13
    {2, 5, 3}, //14
    {2, 6, 3}, //15
    {2, 7, 3}, //16
    {2, 7, 4}, //17
    {3, 7, 4}, //18
    {3, 7, 5}, //19
    {4, 7, 5}, //20
    {4, 7, 6}, //21
    {5, 7, 6}, //22
    {6, 7, 6}, //23
    {6, 7, 7}, //24
    {7, 7, 7}, //25
  }, propSeg = bitTimingTable[result][0], pSeg1 = bitTimingTable[result][1], pSeg2 = bitTimingTable[result][2];
  FLEXCANb_CTRL1(_bus) = (FLEXCAN_CTRL_PROPSEG(propSeg) | FLEXCAN_CTRL_RJW(1) | FLEXCAN_CTRL_PSEG1(pSeg1) |
                    FLEXCAN_CTRL_PSEG2(pSeg2) | FLEXCAN_CTRL_ERR_MSK | FLEXCAN_CTRL_PRESDIV(divisor));
  ( listen_only != FLEXCAN_LISTEN_ONLY ) ? FLEXCANb_CTRL1(_bus) &= ~FLEXCAN_CTRL_LOM : (FLEXCANb_CTRL1(_bus) |= FLEXCAN_CTRL_LOM); /* listen-only mode */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

static void setClock(int clock) {
  if ( clock == CLK_OFF ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(3) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_8MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(9);
  if ( clock == CLK_16MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(4);
  if ( clock == CLK_20MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(3);
  if ( clock == CLK_24MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(1) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_40MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(1);
  if ( clock == CLK_60MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(0);

  setBaudRate(currentBitrate, (( FLEXCANb_CTRL1(_bus) & FLEXCAN_CTRL_LOM ) ? FLEXCAN_LISTEN_ONLY : FLEXCAN_TX));
}

static void enableFIFOInterrupt(bool status) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN) ) return; /* FIFO must be enabled first */
  if ( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) return; /* FIFO interrupts already enabled */
  FLEXCANb_IMASK1(_bus) &= ~0xFF; /* disable FIFO interrupt flags */
  if ( status ) FLEXCANb_IMASK1(_bus) |= FLEXCAN_IMASK1_BUF5M; /* enable FIFO interrupt */
}

static void enableMBInterrupt(int8_t mb_num, bool status) {
  if ( mb_num < mailboxOffset() ) return; /* mailbox not available */
  if ( status ) writeIMASKBit(mb_num,1); /* enable mailbox interrupt */
  else if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) writeIMASKBit(mb_num,1); /* transmit interrupt keeper */
  else writeIMASKBit(mb_num, 0); /* disable mailbox interrupt */
}

static void enableMBInterrupts(bool status) {
  FLEXCAN_EnterFreezeMode();
  for ( uint8_t mb_num = mailboxOffset(); mb_num < FLEXCANb_MAXMB_SIZE(_bus); mb_num++ ) {
    enableMBInterrupt((int)mb_num, status);
  }
  FLEXCAN_ExitFreezeMode();
}

static void enableFIFO(bool status) {
  bool frz_flag_negate = !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_FEN; // Disable FIFO if already enabled for cleanup.
  writeIMASK(0ULL); // disable all FIFO/MB Interrupts

  for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_bus); i++ ) { // clear all mailboxes
    volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (i * 0x10)));
    mbxAddr[0] = mbxAddr[1] = mbxAddr[2] = mbxAddr[3] = 0; // code, id, word0, word1
    FLEXCANb_RXIMR(_bus, i) = 0UL; // CLEAR MAILBOX MASKS (RXIMR)
  }

  FLEXCANb_RXMGMASK(_bus) = FLEXCANb_RXFGMASK(_bus) = 0;
  writeIFLAG(readIFLAG()); // (all bits reset when written back)

  if ( status ) {
    FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FEN;
    for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
      FLEXCANb_MBn_CS(_bus,i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
      enableMBInterrupt((int)i,1); /* enable TX interrupt */
    } 
  } 
  else { // FIFO disabled default setup of mailboxes, 0-7 RX, 8-15 TX
    for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_bus); i++ ) { // clear all mailboxes
      volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (i * 0x10)));
      if ( i < (FLEXCANb_MAXMB_SIZE(_bus) / 2) ) {
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | ((i < (FLEXCANb_MAXMB_SIZE(_bus) / 4)) ? 0 : FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR);
        FLEXCANb_RXIMR(_bus, i) = 0UL | ((FLEXCANb_CTRL2(_bus) & FLEXCAN_CTRL2_EACEN) ? (1UL << 30) : 0); // (RXIMR)
      }
      else {
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
        enableMBInterrupt((int)i,1); /* enable TX interrupt */
      }
    }
  }
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

static void setMBFilterProcessing(FLEXCAN_MAILBOX mb_num, uint32_t filter_id, uint32_t calculated_mask) {
  bool frz_flag_negate = !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_RXIMR(_bus, mb_num) = calculated_mask | ((FLEXCANb_CTRL2(_bus) & FLEXCAN_CTRL2_EACEN) ? (1UL << 30) : 0);
  FLEXCANb_MBn_ID(_bus, mb_num) = ((!(FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE)) ? FLEXCAN_MB_ID_IDSTD(filter_id) : FLEXCAN_MB_ID_IDEXT(filter_id));
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

static void fifo_filter_store(int type, uint8_t filter, uint32_t id_count, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t id5) {
  fifo_filter_table[filter][0] = (fifo_filter_table[filter][0] & 0xF0000) | filter; // first 7 bits reserved for fifo filter
  fifo_filter_table[filter][0] |= (id_count << 7); // we store the quantity of ids after the fifo filter count 
  /* bit 16-19: extended ids */
  /* bit 28: filter enabled */
  fifo_filter_table[filter][0] |= (type << 29); // we reserve 3 upper bits for type
  fifo_filter_table[filter][1] = id1; // id1
  fifo_filter_table[filter][2] = id2; // id2
  fifo_filter_table[filter][3] = id3; // id3
  fifo_filter_table[filter][4] = id4; // id4
  fifo_filter_table[filter][5] = id5; // id5
}

static void filter_store(int type, FLEXCAN_MAILBOX mb_num, uint32_t id_count, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t id5) {
  mb_filter_table[mb_num][0] = mb_num; // first 7 bits reserved for MB
  mb_filter_table[mb_num][0] |= (id_count << 7); // we store the quantity of ids after the mailboxes 
  /* bit 28: filter enabled */
  mb_filter_table[mb_num][0] |= (type << 29); // we reserve 3 upper bits for type
  volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (mb_num * 0x10)));
  mb_filter_table[mb_num][0] |= ( ((mbxAddr[0] & 0x600000) ? 1UL : 0UL) << 27); /* extended flag check */
  mb_filter_table[mb_num][1] = id1; // id1
  mb_filter_table[mb_num][2] = id2; // id2
  mb_filter_table[mb_num][3] = id3; // id3
  mb_filter_table[mb_num][4] = id4; // id4
  mb_filter_table[mb_num][5] = id5; // id5
}

volatile bool filter_match(int8_t mb_num, uint32_t id) {
  if ( !(mb_filter_table[mb_num][0] & 0x10000000) ) return 1;
  if ( (mb_filter_table[mb_num][0] >> 29) == FLEXCAN_MULTI ) {
    for ( uint8_t i = 0; i < ((mb_filter_table[mb_num][0] & 0x380) >> 7); i++) if ( id == mb_filter_table[mb_num][i+1] ) return 1;
  }
  else if ( (mb_filter_table[mb_num][0] >> 29) == FLEXCAN_RANGE ) {
    if ( id >= mb_filter_table[mb_num][1] && id <= mb_filter_table[mb_num][2] ) return 1;
  }
  return 0;
}

bool setMBFilter(int8_t mb_num, uint32_t id1, uint32_t id2) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return 0; /* exit on TX mailbox */ 
  uint32_t mask = ( !(FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE) ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2) ^ (id1 & id2)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2) ^ (id1 & id2)) ^ 0x1FFFFFFF);
  setMBFilterProcessing(mb_num,id1,mask);
  filter_store(FLEXCAN_MULTI, mb_num, 2, id1, id2, 0, 0, 0);
  return 1;
}

void writeTxMailbox(int8_t mb_num, CAN_message_t *msg) {
  writeIFLAGBit(mb_num);
  uint32_t code = 0;
  volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (mb_num * 0x10)));
  mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  mbxAddr[1] = (( msg->flags.extended ) ? ( msg->id & FLEXCAN_MB_ID_EXT_MASK ) : FLEXCAN_MB_ID_IDSTD(msg->id));
  if ( msg->flags.remote ) code |= (1UL << 20);
  if ( msg->flags.extended ) code |= (3UL << 21);
  for ( uint8_t i = 0; i < (8 >> 2); i++ ) mbxAddr[2 + i] = (msg->buf[0 + i * 4] << 24) | (msg->buf[1 + i * 4] << 16) | (msg->buf[2 + i * 4] << 8) | msg->buf[3 + i * 4];
  code |= msg->len << 16;
  mbxAddr[0] = code | FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
}

int mailboxOffset() {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) ) return 0; /* return offset 0 since FIFO is disabled */
  uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_bus) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
  if ( FLEXCANb_MAXMB_SIZE(_bus) < (6 + ((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
  return (FLEXCANb_MAXMB_SIZE(_bus) - remaining_mailboxes); /* otherwise return offset MB position after FIFO area */
}

void softReset() {
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_SOFT_RST;
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_SOFT_RST);
}

void struct2queueRx(CAN_message_t *cl) {
  CANListener *thisListener;
  for (uint8_t listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++) {
    thisListener = listener[listenerPos];
    if (thisListener != 0) {
      if (thisListener->callbacksActive & (1UL << cl->mb)) thisListener->frameHandler (cl, cl->mb);
      if (thisListener->generalCallbackActive) thisListener->frameHandler (cl, -1);
    }
  }
}

int readFIFO(CAN_message_t *msg) {
  //delayMicroseconds(150);
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN) ) return 0; /* FIFO is disabled */
  if ( !(FLEXCANb_MCR(_bus) & (1UL << 15)) ) { /* if DMA is not enabled, check interrupt flag, else continue. */
    if ( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) return 0; /* FIFO interrupt enabled, polling blocked */
  }
  if ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF5I ) { /* message available */
    volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80));
    uint32_t code = mbxAddr[0];
    msg->len = (code & 0xF0000) >> 16;
    msg->flags.remote = (bool)(code & (1UL << 20));
    msg->flags.extended = (bool)(code & (1UL << 21));
    msg->timestamp = code & 0xFFFF;
    msg->id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg->flags.extended) ? 0 : 18);
    uint32_t data0 = mbxAddr[2]; 
    for ( int8_t d = 0; d < 4 ; d++ ) msg->buf[3 - d] = (uint8_t)(data0 >> (8 * d));
    uint32_t data1 = mbxAddr[3];
    for ( int8_t d = 0; d < 4 ; d++ ) msg->buf[7 - d] = (uint8_t)(data1 >> (8 * d));
    // msg->bus = busNumber;
    msg->idhit = code >> 23;
    msg->mb = FIFO; /* store the mailbox the message came from (for callback reference) */
    if ( !(FLEXCANb_MCR(_bus) & (1UL << 15)) ) writeIFLAGBit(5); /* clear FIFO bit only, NOT FOR DMA USE! */
    // frame_distribution(msg);
    // if ( fifo_filter_match(msg->id) ) 
    return 1; /* message available */
  }
  return 0; /* message not available */
}

int readMB(CAN_message_t *msg) {
  uint64_t iflag = 0;
  for ( uint8_t cycle_limit = 3, mailboxes = mailboxOffset(); mailbox_reader_increment <= FLEXCANb_MAXMB_SIZE(_bus); ++mailbox_reader_increment ) {
    iflag = readIFLAG();
    if ( iflag && (mailbox_reader_increment >= (64 - __builtin_clzll(iflag))) ) { /* break from MSB's if unset, add 1 to prevent undefined behaviour in clz for 0 check */
      mailbox_reader_increment = mailboxOffset();
      if ( !--cycle_limit ) return 0;
    }
    if ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) {  /* FIFO is enabled, get only remaining RX (if any) */
        if ( mailbox_reader_increment < mailboxes ) mailbox_reader_increment = mailboxes - 1; /* go back to position end of fifo+filter region */
    }
    if ( mailbox_reader_increment >= FLEXCANb_MAXMB_SIZE(_bus) ) {
      mailbox_reader_increment = mailboxOffset();
      if ( !--cycle_limit ) return 0;
    }
    volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (mailbox_reader_increment * 0x10)));
    if ((readIMASK() & (1ULL << mailbox_reader_increment))) continue; /* don't read interrupt enabled mailboxes */
    uint32_t code = mbxAddr[0];
    if ( (FLEXCAN_get_code(code) >> 3) ) continue; /* skip TX mailboxes */
    //if (!(code & 0x600000) && !(iflag & (1ULL << mailbox_reader_increment))) continue; /* don't read unflagged mailboxes, errata: extended mailboxes iflags do not work in poll mode, must check CS field */
    if ( ( FLEXCAN_get_code(code) == FLEXCAN_MB_CODE_RX_FULL ) ||
         ( FLEXCAN_get_code(code) == FLEXCAN_MB_CODE_RX_OVERRUN ) ) {
      msg->flags.remote = (bool)(code & (1UL << 20));
      msg->flags.extended = (bool)(code & (1UL << 21));
      msg->id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg->flags.extended) ? 0 : 18);
      if ( FLEXCAN_get_code(code) == FLEXCAN_MB_CODE_RX_OVERRUN ) msg->flags.overrun = 1;
      msg->len = (code & 0xF0000) >> 16;
      msg->mb = mailbox_reader_increment++;
      msg->timestamp = code & 0xFFFF;
      // msg->bus = busNumber;
      for ( uint8_t i = 0; i < (8 >> 2); i++ ) for ( int8_t d = 0; d < 4 ; d++ ) msg->buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
      mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | ((msg->flags.extended) ? (FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE) : 0);
      (void)FLEXCANb_TIMER(_bus);
      writeIFLAGBit(msg->mb);
      // frame_distribution(msg);
      // if ( filter_match((int)msg->mb, msg->id) ) 
      return 1;
    }
  } 
  return 0; /* no messages available */
}

int read(CAN_message_t *msg) {
  bool _random = random();
  if ( ( !_random ) && ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) &&
       !( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) &&
       ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF5I ) ) return readFIFO(msg);
  return readMB(msg);
}

int write_mb(FLEXCAN_MAILBOX mb_num, CAN_message_t *msg) {
  if ( mb_num < mailboxOffset() ) return 0; /* FIFO doesn't transmit */
  volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (mb_num * 0x10)));
  if ( !((FLEXCAN_get_code(mbxAddr[0])) >> 3) ) return 0; /* not a transmit mailbox */
    int first_tx_mb = getFirstTxBox();
    if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, first_tx_mb)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
      writeTxMailbox(first_tx_mb, msg);
      return 1; /* transmit entry accepted */
    }
  else {
    msg->mb = first_tx_mb;
    add_buf(&tx_buf,msg); /* queue if no mailboxes found */
    return -1; /* transmit entry failed, no mailboxes available, queued */
  }
  if ( FLEXCAN_get_code(mbxAddr[0]) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
    writeTxMailbox(mb_num, msg);
    return 1;
  }
  msg->mb = mb_num;
  add_buf(&tx_buf, msg); /* queue if no mailboxes found */
  return -1; /* transmit entry failed, no mailboxes available, queued */
}

int write(CAN_message_t *msg) {
  int first_tx_mb = getFirstTxBox();
  if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, first_tx_mb)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
    writeTxMailbox(first_tx_mb, msg);
    return 1; /* transmit entry accepted */
  }
  else {
    msg->mb = first_tx_mb;
    add_buf(&tx_buf,msg); /* queue if no mailboxes found */
    return -1; /* transmit entry failed, no mailboxes available, queued */
  }
  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
    if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
      writeTxMailbox(i, msg);
      return 1; /* transmit entry accepted */
    }
  }
  msg->mb = -1;
  add_buf(&tx_buf,msg); /* queue if no mailboxes found */
  return -1; /* transmit entry failed, no mailboxes available, queued */
}

// void onReceive_Mb(int8_t mb_num, _MB_ptr handler) {
//   if ( FIFO == mb_num ) {
//     _mbHandlers[0] = handler;
//     return;
//   }
//   _mbHandlers[mb_num] = handler;
// }

// void onReceive(_MB_ptr handler) {
//   _mainHandler = handler;
// }

void onTransmit_Mb(int8_t mb_num, _MB_ptr handler) {
  if ( FIFO == mb_num ) {
    _mbTxHandlers[0] = handler;
    return;
  }
  _mbTxHandlers[mb_num] = handler;
}

void onTransmit(_MB_ptr handler) {
  _mainTxHandler = handler;
}

void mbCallbacks(int8_t mb_num, CAN_message_t *msg) {
  if ( mb_num == FIFO ) {
    if ( _mbHandlers[0] ) _mbHandlers[0](msg);
    if ( _mainHandler ) _mainHandler(msg);
    return;
  }
  if ( _mbHandlers[mb_num] ) _mbHandlers[mb_num](msg);
  if ( _mainHandler ) _mainHandler(msg);
}

void canbus_execute_buf(sys_state_t state)
{
  spin_lock = true;
  int txBox = mailboxOffset();
  // for (;;)
  // {
    for (int i = txBox ; i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
      if (txBox >= FLEXCANb_MAXMB_SIZE(_bus) || !(tx_buf.count) || !canbus_On) 
        break;
      CAN_message_t tx_msg;// = {0};
      remove_buf(&tx_buf,&tx_msg);
      if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) == FLEXCAN_MB_CODE_TX_INACTIVE ){
        writeTxMailbox(i, &tx_msg);
        txBox++;
        break;
      }
    }
  // }
  // if (tx_buf.count)
    // protocol_enqueue_rt_command(canbus_execute_buf);
  spin_lock = false;
}

void canbus_events(uint_fast16_t state) {
  // UNUSED(state);
  // static int maxCount;
  int bufCount = tx_buf.count;
  if (bufCount)
    canbus_execute_buf(state);
  // if (bufCount > maxCount){ 
    // maxCount = bufCount;
    // char msg_c[30];
    // sprintf_P(msg_c,"CAN Buffer tx=%u max=%u",bufCount,maxCount);
    // report_message(msg_c,Message_Info);
  // }
  // if(maxCount && !canbus_On)
    // maxCount = bufCount = 0;
  // if (sync_msg.active && sync_msg.msg)
  //   report_Frame(sync_msg.msg);
  on_execute_realtime(state);
}

void flexcan_interrupt(void) {
  CAN_message_t msg; // setup a temporary storage buffer
  uint64_t imask = readIMASK(), iflag = readIFLAG();

  if ( !(FLEXCANb_MCR(_bus) & (1UL << 15)) ) { /* if DMA is disabled, ONLY THEN you can handle FIFO in ISR */
    if ( (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN) && (imask & FLEXCAN_IMASK1_BUF5M) && (iflag & FLEXCAN_IFLAG1_BUF5I) ) { /* FIFO is enabled, capture frames if triggered */
      volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (0 * 0x10)));
      uint32_t code = mbxAddr[0];
      msg.len = (code & 0xF0000) >> 16;
      msg.flags.remote = (bool)(code & (1UL << 20));
      msg.flags.extended = (bool)(code & (1UL << 21));
      msg.timestamp = code & 0xFFFF;
      msg.id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg.flags.extended) ? 0 : 18);
      msg.idhit = code >> 23;
      for ( uint8_t i = 0; i < (8 >> 2); i++ ) for ( int8_t d = 0; d < 4 ; d++ ) msg.buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
      msg.mb = FIFO; /* store the mailbox the message came from (for callback reference) */
      (void)FLEXCANb_TIMER(_bus);
      writeIFLAGBit(5); /* clear FIFO bit only! */
      if ( iflag & FLEXCAN_IFLAG1_BUF6I ) writeIFLAGBit(6); /* clear FIFO bit only! */
      if ( iflag & FLEXCAN_IFLAG1_BUF7I ) writeIFLAGBit(7); /* clear FIFO bit only! */
      // frame_distribution(&msg);
      // if (fifo_filter_match(msg.id))
      if (sync_msg.active && sync_msg.id == msg.id){
        memcpy(&sync_msg.msg->buf,&msg.buf,msg.len);
        sync_msg.ready = true;
        sync_msg.active = false;
        sync_msg.time_end = micros();//hal.get_elapsed_ticks();
        // add_sync_msg(&rx_sync_buf,&msg);
      }
      else {
        mbCallbacks(FIFO,&msg);
        if (_mainHandler)
          _mainHandler(&msg);
        struct2queueRx(&msg);
      }
    }
  }

  uint8_t exit_point = 64 - __builtin_clzll(iflag | 1); /* break from MSB's if unset, add 1 to prevent undefined behaviour in clz for 0 check */
  for ( uint8_t mb_num = mailboxOffset(); mb_num < FLEXCANb_MAXMB_SIZE(_bus); mb_num++ ) {
    if ( mb_num >= exit_point ) break; /* early exit from higher unflagged mailboxes */
    if (!(imask & (1ULL << mb_num))) continue; /* don't read non-interrupt mailboxes */
    if (!(iflag & (1ULL << mb_num))) continue; /* don't read unflagged mailboxes */
    volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(_bus + 0x80 + (mb_num * 0x10)));
    uint32_t code = mbxAddr[0];
    uint32_t mb_code = FLEXCAN_get_code(code);

    if ( ( mb_code == FLEXCAN_MB_CODE_RX_FULL ) || ( mb_code == FLEXCAN_MB_CODE_RX_OVERRUN ) ) {
      msg.flags.extended = (bool)(code & (1UL << 21));
      msg.id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg.flags.extended) ? 0 : 18);
      if ( FLEXCAN_get_code(code) == FLEXCAN_MB_CODE_RX_OVERRUN ) msg.flags.overrun = 1;
      msg.len = (code & 0xF0000) >> 16;
      msg.mb = mb_num;
      msg.timestamp = code & 0xFFFF;
      for ( uint8_t i = 0; i < (8 >> 2); i++ ) for ( int8_t d = 0; d < 4 ; d++ ) msg.buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
      mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | ((msg.flags.extended) ? (FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE) : 0);
      // if (sync_msg.active && sync_msg.id == msg.id){
      //   char msg_c[40];
      //   sprintf(msg_c,"CAN_MB_CODE_RX_FULL mb=%u id=%lu",mb_num, msg.id);
      //   report_message(msg_c,Message_Info);
      //   digitalToggleFast(LED_BUILTIN);
        // msg = (CAN_message_t){0};
        // readMB(&msg);
        // add_sync_msg(&rx_sync_buf,&msg);
      // }
      (void)FLEXCANb_TIMER(_bus);
      writeIFLAGBit(mb_num);
      if (sync_msg.active && sync_msg.id == msg.id){
        memcpy(&sync_msg.msg->buf,&msg.buf,msg.len);
        sync_msg.ready = true;
        sync_msg.active = false;
        sync_msg.time_end = micros();//hal.get_elapsed_ticks();
        // add_sync_msg(&rx_sync_buf,&msg);
      }
      else {
        mbCallbacks(mb_num,&msg);
        struct2queueRx(&msg); /* store frame in queue */
      }
    }

    else if ( mb_code == FLEXCAN_MB_CODE_RX_EMPTY ) {
      /* there are no flags for EMPTY reception boxes, however, when sending remote
         frames, the mailboxes switch to RX_EMPTY and trigger the flag */
      if (!(iflag & (1ULL << mb_num))) continue; /* only process the flagged RX_EMPTY mailboxes */

      msg.flags.extended = (bool)(code & (1UL << 21));
      msg.id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg.flags.extended) ? 0 : 18);
      if ( FLEXCAN_get_code(code) == FLEXCAN_MB_CODE_RX_OVERRUN ) msg.flags.overrun = 1;
      msg.len = (code & 0xF0000) >> 16;
      msg.mb = mb_num;
      msg.timestamp = code & 0xFFFF;
      // msg.bus = busNumber;
      for ( uint8_t i = 0; i < (8 >> 2); i++ ) for ( int8_t d = 0; d < 4 ; d++ ) msg.buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
      // if ( mb_num == FIFO ) {
      //   if ( _mbTxHandlers[0] ) _mbTxHandlers[0](&msg);
      //   if ( _mainTxHandler ) _mainTxHandler(&msg);
      // }
      // else {
      //   if ( _mbTxHandlers[mb_num] ) _mbTxHandlers[mb_num](&msg);
      //   if ( _mainTxHandler ) _mainTxHandler(&msg);
      // }
  
      if (tx_buf.count > 0 && !spin_lock) {
        CAN_message_t frame = {0};
        remove_buf(&tx_buf,&frame);
      //   // int result = 0;
      //   if ( frame.mb == -1 ) {
          writeTxMailbox(mb_num, &frame);
      //     // result = 1;
      //   }
      //   else if ( frame.mb == mb_num ) {
      //     writeTxMailbox(frame.mb, &frame);
      //     // result = 1;
      //   }
        // if (!result) add_buf(&tx_buf,&frame);
      }
      else {
        writeIFLAGBit(mb_num); /* just clear IFLAG if no TX queues exist */
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE); /* set it back to a TX mailbox */
      }
    }

    else if ( mb_code == FLEXCAN_MB_CODE_TX_INACTIVE ) {
      msg.flags.extended = (bool)(code & (1UL << 21));
      msg.id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg.flags.extended) ? 0 : 18);
      if ( FLEXCAN_get_code(code) == FLEXCAN_MB_CODE_RX_OVERRUN ) msg.flags.overrun = 1;
      msg.len = (code & 0xF0000) >> 16;
      msg.mb = mb_num;
      msg.timestamp = code & 0xFFFF;
      // msg.bus = busNumber;
      if (msg.len == 8) for ( uint8_t i = 0; i < (8 >> 2); i++ ) for ( int8_t d = 0; d < 4 ; d++ ) msg.buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
      
      if ( mb_num == FIFO ) {
        if ( _mbTxHandlers[0] ) _mbTxHandlers[0](&msg);
        if ( _mainTxHandler ) _mainTxHandler(&msg);
      }
      
      // if (sync_msg.active && sync_msg.id == msg.id){
      //   memcpy(&sync_msg.msg->buf,&msg.buf,msg.len);
      //   sync_msg.ready = true;
      //   sync_msg.active = false;
      //   sync_msg.time_end = micros();//hal.get_elapsed_ticks();
      //   // add_sync_msg(&rx_sync_buf,&msg);
      // }
      // else
       if (tx_buf.count > 0 && !spin_lock) {
        CAN_message_t frame = {0};
        remove_buf(&tx_buf,&frame);
        writeTxMailbox(mb_num, &frame);
      }
      else {
        (void)FLEXCANb_TIMER(_bus);
        writeIFLAGBit(mb_num); /* just clear IFLAG if no TX queues exist */
      }
    }
  
    else if ( mb_code == FLEXCAN_MB_CODE_TX_RESPONSE  )  {
      msg.flags.extended = (bool)(code & (1UL << 21));
      msg.id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg.flags.extended) ? 0 : 18);
      // if ( FLEXCAN_get_code(code) == FLEXCAN_MB_CODE_RX_OVERRUN ) msg.flags.overrun = 1;
      msg.len = (code & 0xF0000) >> 16;
      msg.mb = mb_num;
      msg.timestamp = code & 0xFFFF;
      // msg.bus = busNumber;
      for ( uint8_t i = 0; i < (8 >> 2); i++ ) for ( int8_t d = 0; d < 4 ; d++ ) msg.buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
      mbxAddr[0] = msg.len << 16 | FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO) | ((msg.flags.extended) ? (FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE) : 0);
      // char msg_c[30];
      // sprintf(msg_c,"CAN_MB_CODE_TX_RESPONSE mb=%u",mb_num);
      // report_message(msg_c,Message_Info);
      // digitalToggleFast(LED_BUILTIN);
      // report_Frame(&msg);
      // add_sync_msg(&rx_sync_buf,&msg);
      (void)FLEXCANb_TIMER(_bus);
      writeIFLAGBit(mb_num);
      if (sync_msg.active && sync_msg.id == msg.id){
        memcpy(&sync_msg.msg->buf,&msg.buf,msg.len);
        sync_msg.ready = true;
        sync_msg.active = false;
        sync_msg.time_end = micros();//hal.get_elapsed_ticks();
        // add_sync_msg(&rx_sync_buf,&msg);
      }
      // if (tx_buf.count > 0) {
        // CAN_message_t frame = {0};
        // remove_buf(&tx_buf,&frame);
        // writeTxMailbox(mb_num, &frame);
      // }
      // else {
      // FLEXCANb_MBn_CS(_bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO);
      // writeIFLAGBit(mb_num); /* just clear IFLAG if no TX queues exist */
      // }
    }
    // if (sync_msg.active && sync_msg.mb == mb_num){
    //   digitalToggleFast(LED_BUILTIN);
    //   add_sync_msg(&rx_sync_buf,&msg);
    // }
  }
      
  uint32_t reg_esr = FLEXCANb_ESR1(_bus);
  bool _state =  reg_esr&0x40000 && !(reg_esr&0xF000) && !(reg_esr&0x30) ? true : false; /* synch, no crc/ack/bit errors */
  if (canbus_On != _state) 
      canbus_On = _state;

  FLEXCANb_ESR1(_bus) |= reg_esr;
  asm volatile ("dsb");	
}

bool canbus_attachObj (CANListener *_listener) {
  for (uint8_t i = 0; i < SIZE_LISTENERS; i++) {
    if (listener[i] == 0) {
      listener[i] = _listener;
      // listener[i]->callbacksActive = 0;
      return true;
    }
  }
  return false;
}

bool canbus_detachObj (CANListener *_listener) {
  for (uint8_t i = 0; i < SIZE_LISTENERS; i++) {
    if (listener[i] == _listener) {
      listener[i] = 0;
      return true;
    }
  }
  return false;
}

void can_init() {
  //Set pins
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 =  0x10; // pin 1 T4B1+B2
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 = 0x10B0; // pin 1 T4B1+B2
  IOMUXC_FLEXCAN2_RX_SELECT_INPUT = 0x01;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x10; // pin 0 T4B1+B2
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 = 0x10B0; // pin 0 T4B1+B2
  //Set interupt isr
  NVIC_SET_PRIORITY(nvicIrq,12);
  NVIC_DISABLE_IRQ(IRQ_CAN2);
  CCM_CCGR0 |= CCM_CCGR0_CAN2(CCM_CCGR_ON) | CCM_CCGR0_CAN2_SERIAL(CCM_CCGR_ON); 
  //Reset Framebuffers
  init_buf(&tx_buf, SIZE_TX_BUFFER);
  init_sync_buf(&rx_sync_buf,SIZE_RX_SYNC_BUFFER);
  // Reset Listeners
  for (uint8_t i = 0; i < SIZE_LISTENERS; i++) {listener[i] = 0;}
  if ( !getClock() ) 
    setClock(CLK_24MHz); /* no clock enabled, enable osc clock */
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_MDIS; /* enable module */
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_CTRL1(_bus) |= FLEXCAN_CTRL_LOM; /* listen only mode */
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FRZ; /* enable freeze bit */
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_LPM_ACK);
  softReset(); /* reset bus */
  while (!(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK));

  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_SRX_DIS; /* Disable self-reception */
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_IRMQ; // individual mailbox masking
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_AEN; // TX ABORT FEATURE
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_LPRIO_EN; // TX PRIORITY FEATURE
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_SLF_WAK; // SELF-WAKE UP FEATURE	
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_WAK_SRC; // WAKE-UP LOW-PASS FILTER
  FLEXCANb_MCR(_bus) &= ~0x8800; // disable DMA and FD (valid bits are reserved in legacy controllers)

  // FLEXCANb_CTRL2(_bus) |= FLEXCAN_CTRL2_RRS; // store remote frames
  FLEXCANb_CTRL2(_bus) |= FLEXCAN_CTRL2_EACEN; /* handles the way filtering works. Library adjusts to whether you use this or not */ 
  FLEXCANb_CTRL2(_bus) |= FLEXCAN_CTRL2_MRP; // mailbox > FIFO priority.
  // FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_WRN_EN;
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_WAK_MSK;
 
  enableFIFO(0); /* clears all data and layout to legacy mailbox mode */
  FLEXCAN_ExitFreezeMode();
  attachInterruptVector(nvicIrq,flexcan_interrupt);
  NVIC_ENABLE_IRQ(nvicIrq);
  initOK = true;
}

void canbus_begin(_MB_ptr handler, uint32_t baudrate) {
  if (!initOK){
    can_init();
  }
  else {
    enableFIFO(0);
  }
  _mainHandler = handler;
  if (currentBitrate != baudrate)
    setBaudRate(baudrate,FLEXCAN_TX);
  // FLEXCAN_EnterFreezeMode();
  // for ( uint8_t mb_num = 0; mb_num < FLEXCANb_MAXMB_SIZE(_bus); mb_num++ ) {
  //   enableMBInterrupt((int)mb_num, 1);
  // }
  // FLEXCAN_ExitFreezeMode();
  // enableMBInterrupts(1);

  enableFIFO(1);
  enableFIFOInterrupt(1);
}

bool canbus_connected() {
  return canbus_On;
}

CAN_sync_message_t *canbus_write_sync_msg(CAN_message_t *msg, bool enable){
  if (!enable){
    sync_msg = (CAN_sync_message_t){0};
    // writeIFLAGBit(sync_msg.mb);
    // FLEXCANb_MBn_CS(_bus, sync_msg.mb) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
    // (void)FLEXCANb_TIMER(_bus);
    // sync_msg = (CAN_sync_message_t){0};
    return NULL;
  }

  int mbox = getFirstTxBox();
  uint32_t mb_code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mbox));
  if (mb_code == FLEXCAN_MB_CODE_TX_INACTIVE){
    sync_msg.active = true;
    sync_msg.mb = mbox;
    sync_msg.id = msg->id;
    sync_msg.time = micros();//hal.get_elapsed_ticks();
    // sync_msg.msg_prev = sync_msg.msg;// NULL;
    sync_msg.msg = msg;
    writeTxMailbox(mbox,msg);
  }
  // char msg_c[30];
  // sprintf(msg_c,"CAN sync msg mb=%u",mbox);
  // report_message(msg_c,Message_Info);
  return &sync_msg;
}

int canbus_write_blocking(CAN_message_t *msg, bool block)
{
  if (!block){
    return add_buf(&tx_buf,msg) ? 2 : 0;
  }
  // find an available buffer
  int buffer = 0;
  // while (spin_lock);
  for ( int index = getFirstTxBox(); ; ) {
    uint32_t mb_code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, index));
    if ((mb_code == FLEXCAN_MB_CODE_TX_INACTIVE) || (mb_code == FLEXCAN_MB_CODE_RX_EMPTY)) {
      buffer = index;
      break;// found one
    }
    index++;
    if (index > FLEXCANb_MAXMB_SIZE(_bus)) {
      return add_buf(&tx_buf,msg) ? 2 : 0;
    }
  }
  writeTxMailbox(buffer,msg);
  return 1;
  // transmit the frame
  // FLEXCANb_MBn_CS(_bus, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  // if(msg->flags.extended) {
  //   FLEXCANb_MBn_ID(_bus, buffer) = (msg->id & FLEXCAN_MB_ID_EXT_MASK);
  // } else {
  //   FLEXCANb_MBn_ID(_bus, buffer) = FLEXCAN_MB_ID_IDSTD(msg->id);
  // }
  // if (msg->flags.remote){
  //   FLEXCANb_MBn_WORD0(_bus, buffer) = 0UL;
  //   FLEXCANb_MBn_WORD1(_bus, buffer) = 0UL;
  // }
  // else {
  //   FLEXCANb_MBn_WORD0(_bus, buffer) = (msg->buf[0]<<24)|(msg->buf[1]<<16)|(msg->buf[2]<<8)|msg->buf[3];
  //   FLEXCANb_MBn_WORD1(_bus, buffer) = (msg->buf[4]<<24)|(msg->buf[5]<<16)|(msg->buf[6]<<8)|msg->buf[7];
  // }
  // uint32_t code = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
  // if(msg->flags.extended) 
  //   code |= FLEXCAN_MB_CS_LENGTH(msg->len) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  // else if(msg->flags.remote) 
  //   code |= (1UL << 20);

  // FLEXCANb_MBn_CS(_bus, buffer) |= code | FLEXCAN_MB_CS_LENGTH(msg->len);
  // FLEXCANb_IFLAG1(_bus) |= (1UL << buffer);

  // return 1;
}

int canbus_write(CAN_message_t *msg){
  for (int8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
    if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
      writeTxMailbox(i, msg);
      return 1; /* transmit entry accepted */
    }
  }
  CAN_message_t msg_copy;
  memcpy(&msg_copy,msg,sizeof(CAN_message_t));
  msg_copy.mb = -1;
  add_buf(&tx_buf, &msg_copy); /* queue if no mailboxes found */
  return -1; /* transmit entry failed, no mailboxes available, queued */
}

void onReportOptions (bool newopt)
{
    on_report_options(newopt);
  	// bool isDMA = (bool)FLEXCANb_MCR(_bus) & (1UL << 15);
    if(!newopt){
      uint16_t clk = getClock();
      char c_msg[70];
      sprintf(c_msg,"[PLUGIN:CANBUS v0.01 Clock=%uMhz Baudrate=%luK txmb=%u maxmb=%lu]",clk,currentBitrate / 1000,mailboxOffset(),FLEXCANb_MAXMB_SIZE(_bus));
      hal.stream.write(c_msg);
      hal.stream.write(ASCII_EOL);
     // hal.stream.write("[PLUGIN:CANBUS v0.01]" ASCII_EOL);
    }
}

void canbus_init()
{
  currentBitrate = 250000;
  can_init();
  on_report_options = grbl.on_report_options;
  grbl.on_report_options = onReportOptions;
	on_execute_realtime = grbl.on_execute_realtime;
  grbl.on_execute_realtime = canbus_events;
  // pinMode(LED_BUILTIN,OUTPUT);
  // digitalToggleFast(LED_BUILTIN);
}

#endif
