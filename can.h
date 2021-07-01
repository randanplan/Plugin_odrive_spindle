
#ifndef  CAN_H__
#define CAN_H__

#include "driver.h"

// Anonymous enum for defining the most common CAN baud rates
typedef enum CAN_baudrates_t{
    CAN_BAUD_125K =   125000,
    CAN_BAUD_250K =   250000,
    CAN_BAUD_500K =   500000,
    CAN_BAUD_1000K =  1000000,
    CAN_BAUD_1M =     1000000
}CAN_baudrates_t;

typedef struct CAN_message_t {
  uint32_t id;          // can identifier
  uint16_t timestamp;   // FlexCAN time when message arrived
  uint8_t idhit; // filter that id came from
  struct {
    bool extended; // identifier is extended (29-bit)
    bool remote;  // remote transmission request packet type
    bool overrun; // message overrun
    bool reserved;
  } flags;
  uint8_t len;      // length of data
  union { // 64 bits - lots of ways to access
      uint8_t buf[8];
	    struct {
            uint32_t low; 
            uint32_t high;
        };
        // uint64_t value;
  };
  // uint16_t timeout;
  int8_t mb;       // used to identify mailbox reception
  uint8_t bus;      // used to identify where the message came from when events() is used.
  bool seq;         // sequential frames
} CAN_message_t;

typedef struct CAN_sync_message_t{
  volatile bool active;
  volatile bool ready;
  int8_t mb;
  uint32_t id;
  uint32_t time;
  uint32_t time_end;
  CAN_message_t *msg;
}CAN_sync_message_t;

typedef enum FLEXCAN_MAILBOX {
  MB0 = 0,  MB1 = 1,  MB2 = 2,  MB3 = 3,  MB4 = 4,  MB5 = 5,  MB6 = 6,  MB7 = 7,  MB8 = 8,
  MB9 = 9,  MB10 = 10,  MB11 = 11,  MB12 = 12,  MB13 = 13,  MB14 = 14,  MB15 = 15,  MB16 = 16,
  MB17 = 17,  MB18 = 18,  MB19 = 19,  MB20 = 20,  MB21 = 21,  MB22 = 22,  MB23 = 23,  MB24 = 24,
  MB25 = 25,  MB26 = 26,  MB27 = 27,  MB28 = 28,  MB29 = 29,  MB30 = 30,  MB31 = 31,  MB32 = 32,
  MB33 = 33,  MB34 = 34,  MB35 = 35,  MB36 = 36,  MB37 = 37,  MB38 = 38,  MB39 = 39,  MB40 = 40,
  MB41 = 41,  MB42 = 42,  MB43 = 43,  MB44 = 44,  MB45 = 45,  MB46 = 46,  MB47 = 47,  MB48 = 48,
  MB49 = 49,  MB50 = 50,  MB51 = 51,  MB52 = 52,  MB53 = 53,  MB54 = 54,  MB55 = 55,  MB56 = 56,
  MB57 = 57,  MB58 = 58,  MB59 = 59,  MB60 = 60,  MB61 = 61,  MB62 = 62,  MB63 = 63,  FIFO = 99
} FLEXCAN_MAILBOX;

typedef struct CANListener{
    uint64_t callbacksActive;
    bool generalCallbackActive;
    void (*frameHandler)(CAN_message_t *msg, int mb);
    void (*txHandler)(int , uint8_t );
}CANListener;

typedef void (*_MB_ptr)(CAN_message_t *msg); /* mailbox / global callbacks */

extern void canbus_events(uint_fast16_t state);

extern void canbus_init();
extern void canbus_begin(_MB_ptr handler, uint32_t baudrate);
extern bool canbus_connected();
extern int canbus_write(CAN_message_t *msg);
extern int canbus_write_blocking(CAN_message_t *msg, bool block);
extern CAN_sync_message_t *canbus_write_sync_msg(CAN_message_t *msg, bool enable);
extern bool canbus_attachObj (CANListener *_listener);
extern bool canbus_detachObj (CANListener *_listener);

#endif
