/*
 * stk500 bootloader code.
 *
 * Thanks to MikeBland for the initial code.
 * https://github.com/MikeBland/StmMultiBoot
 */

#include "stk500.h"
#include "flash.h"
#include "uart.h"

#define OPTIBOOT_MAJVER 4
#define OPTIBOOT_MINVER 5

uint8_t NotSynced = 1;
uint16_t Buff[256];

uint8_t getch(void) {
  uint8_t ch = 0;
  uart_receive(&ch, 1u);
  return ch;
}

void verifySpace(void) {
  if (getch() != CRC_EOP) {
    NotSynced = 1;
    return;
  }
  uart_transmit_ch(STK_INSYNC);
}

void bgetNch(uint8_t count) {
  do {
    getch();
  } while (--count);
  verifySpace();
}

static int8_t stk500_update(void) {
  uint32_t address = 0;
  uint8_t ch, GPIOR0;

  uart_transmit_ch(STK_INSYNC);

  for (;;) {
    /* get character from UART */
    ch = getch();
    if (ch == STK_GET_PARAMETER) {
      GPIOR0 = getch();
      verifySpace();
      if (GPIOR0 == 0x82) {
        uart_transmit_ch(OPTIBOOT_MINVER);
      } else if (GPIOR0 == 0x81) {
        uart_transmit_ch(OPTIBOOT_MAJVER);
      } else {
        /*
         * GET PARAMETER returns a generic 0x03 reply for
         * other parameters - enough to keep Avrdude happy
         */
        uart_transmit_ch(0x03);
      }
    } else if (ch == STK_SET_DEVICE) {
      // SET DEVICE is ignored
      bgetNch(20);
    } else if (ch == STK_SET_DEVICE_EXT) {
      // SET DEVICE EXT is ignored
      bgetNch(5);
    } else if (ch == STK_LOAD_ADDRESS) {
      // LOAD ADDRESS
      uint16_t newAddress;
      newAddress = getch();
      newAddress = (newAddress & 0xff) | (getch() << 8);
      address = newAddress; // Convert from word address to byte address
      address <<= 1;
      verifySpace();
    } else if (ch == STK_UNIVERSAL) {
      // UNIVERSAL command is ignored
      bgetNch(4);
      uart_transmit_ch(0x00);
    } else if (ch == STK_PROG_PAGE) {
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      uint8_t *bufPtr;
      uint16_t length;
      uint16_t count;
      uint8_t *memAddress;
      length = getch() << 8; /* getlen() */
      length |= getch();
      getch(); // discard flash/eeprom byte
      // While that is going on, read in page contents
      count = length;
      bufPtr = (uint8_t *)Buff;
      do {
        *bufPtr++ = getch();
      } while (--count);
      if (length & 1) {
        *bufPtr = 0xFF;
      }
      count = length;
      count += 1;
      count /= 2;
      memAddress = (uint8_t *)(address + FLASH_BASE);

      if ((uint32_t)memAddress < FLASH_BANK1_END) {
        // Read command terminator, start reply
        verifySpace();

        if ((uint32_t)memAddress >= FLASH_APP_START_ADDRESS) {
          if (((uint32_t)memAddress & (FLASH_PAGE_SIZE - 1)) == 0) {
            // At page start so erase it
            flash_erase((uint32_t)memAddress);
          }
          flash_write_halfword((uint32_t)memAddress, Buff, count);
        }
      } else {
        verifySpace();
      }
    } else if (ch == STK_READ_PAGE) {
      uint16_t length;
      uint8_t xlen;
      uint8_t *memAddress;
      memAddress = (uint8_t *)(address + FLASH_BASE);
      // READ PAGE - we only read flash
      xlen = getch(); /* getlen() */
      length = getch() | (xlen << 8);
      getch();
      verifySpace();
      do {
        uart_transmit_ch(*memAddress++);
      } while (--length);

    } else if (ch == STK_READ_SIGN) {
      // READ SIGN - return what Avrdude wants to hear
      verifySpace();
      uart_transmit_ch(SIGNATURE_0);
      if (/*Port*/ 0) {
        uart_transmit_ch(SIGNATURE_3);
        uart_transmit_ch(SIGNATURE_4);
      } else {
        uart_transmit_ch(SIGNATURE_1);
        uart_transmit_ch(SIGNATURE_2);
      }
    } else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
      // Adaboot no-wait mod
      verifySpace();
    } else {
      // This covers the response to commands like STK_ENTER_PROGMODE
      verifySpace();
    }
    if (NotSynced) {
      continue;
    }
    uart_transmit_ch(STK_OK);
  }

  return 0;
}

int8_t stk500_check(void) {
  int8_t ret = -1;
  uint8_t iter, bootrequested = 0;
  uint8_t in = 0;

  /* seach possible sync requests to start upload */
  for (iter = 0; iter < 20 && bootrequested < 2; iter++) {
    uart_receive_timeout(&in, 1u, 10);
    if (in == STK_GET_SYNC) {
      uart_receive_timeout(&in, 1u, 10);
      if (in == CRC_EOP) {
        bootrequested++;
      }
    }
  }

  if (bootrequested > 2) {
    /* Valid update request received. Jump to flash mode */
    ret = stk500_update();
  }

  return ret;
}
