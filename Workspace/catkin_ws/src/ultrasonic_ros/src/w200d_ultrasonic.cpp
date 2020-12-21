/*
 * w200d_ultrasonic.c
 *
 * Created on: Jun 03, 2020 11:08
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "ultrasonic_ros/w200d_ultrasonic.h"

#include <cstring>

const uint8_t FRAME_SOF1 = 0x55;
const uint8_t FRAME_SOF2 = 0x54;
const uint8_t FRAME_EOF1 = 0xff;
const uint8_t FRAME_EOF2 = 0x00;

typedef enum {
  WAIT_FOR_SOF1 = 0,
  WAIT_FOR_SOF2,
  WAIT_FOR_SEN_NUM,
  WAIT_FOR_PAYLOAD,
  WAIT_FOR_EOF1,
  WAIT_FOR_EOF2,
  WAIT_FOR_CHECKSUM
} W200dDecodeState;

typedef struct {
  uint8_t sensor_num;
  uint8_t payload_size;
  uint8_t payload_buffer[W200D_CHANNEL_NUM];
  uint8_t crc_checksum;
} W200MsgBuffer;

static W200MsgBuffer rx_msg;

static uint8_t CalcBufferedFrameChecksum();

bool W200dDecodeMessage(uint8_t c, W200dUltrasonicMessage *ultrasonic_msg) {
  static uint8_t decode_state = WAIT_FOR_SOF1;

  bool new_frame_parsed = false;
  switch (decode_state) {
    case WAIT_FOR_SOF1: {
      // std::cout << "WAIT_FOR_SOF1" << std::endl;
      if (c == FRAME_SOF1) {
        rx_msg.sensor_num = 0;
        rx_msg.crc_checksum = 0;
        rx_msg.payload_size = 0;
        memset(&(rx_msg.payload_buffer), 0, W200D_CHANNEL_NUM);

        decode_state = WAIT_FOR_SOF2;
      }
      break;
    }
    case WAIT_FOR_SOF2: {
      // std::cout << "WAIT_FOR_SOF2" << std::endl;
      if (c == FRAME_SOF2) {
        decode_state = WAIT_FOR_SEN_NUM;
      } else {
        decode_state = WAIT_FOR_SOF1;
      }
      break;
    }
    case WAIT_FOR_SEN_NUM: {
      // std::cout << "WAIT_FOR_SEN_NUM" << std::endl;
      rx_msg.sensor_num = c;
      decode_state = WAIT_FOR_PAYLOAD;
      break;
    }
    case WAIT_FOR_PAYLOAD: {
      // std::cout << "WAIT_FOR_PAYLOAD" << std::endl;
      rx_msg.payload_buffer[rx_msg.payload_size++] = c;
      if (rx_msg.payload_size == 8) decode_state = WAIT_FOR_EOF1;
      break;
    }
    case WAIT_FOR_EOF1: {
      // std::cout << "WAIT_FOR_EOF1" << std::endl;
      if (c == FRAME_EOF1) {
        decode_state = WAIT_FOR_EOF2;
      } else {
        decode_state = WAIT_FOR_SOF1;
      }
      break;
    }
    case WAIT_FOR_EOF2: {
      // std::cout << "WAIT_FOR_EOF2" << std::endl;
      if (c == FRAME_EOF2) {
        decode_state = WAIT_FOR_CHECKSUM;
      } else {
        decode_state = WAIT_FOR_SOF1;
      }
      break;
    }
    case WAIT_FOR_CHECKSUM: {
      // std::cout << "WAIT_FOR_CHECKSUM" << std::endl;
      rx_msg.crc_checksum = c;
      new_frame_parsed = true;
      decode_state = WAIT_FOR_SOF1;
      break;
    }
    default:
      break;
  }

  if (new_frame_parsed) {
    uint8_t internal_checksum = CalcBufferedFrameChecksum();
    if (rx_msg.crc_checksum == internal_checksum) {
      memcpy(ultrasonic_msg->channels, rx_msg.payload_buffer,
             W200D_CHANNEL_NUM);
    }
    // else
    // {
    //     std::cout << "checksum mismatch, received: " << std::hex <<
    //     (uint32_t)(rx_msg.crc_checksum) << " , expected: " <<
    //     (uint32_t)(internal_checksum) << std::dec << std::endl;
    // }
  }

  return new_frame_parsed;
}

uint8_t CalcBufferedFrameChecksum() {
  uint8_t chksum = 0;
  chksum ^= rx_msg.sensor_num;
  for (int i = 0; i < W200D_CHANNEL_NUM; ++i)
    chksum ^= rx_msg.payload_buffer[i];
  return chksum;
}