/*
 * w200d_ultrasonic.h
 *
 * Created on: Jun 03, 2020 11:08
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef W200D_ULTRASONIC_H
#define W200D_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#define W200D_CHANNEL_NUM 8
typedef struct {
  uint8_t channels[W200D_CHANNEL_NUM] __attribute__((aligned(8)));
} W200dUltrasonicMessage;

bool W200dDecodeMessage(uint8_t ch, W200dUltrasonicMessage *ultrasonic_msg);

#endif /* W200D_ULTRASONIC_H */
