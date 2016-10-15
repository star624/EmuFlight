/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  IO_TAG(PA15), TIM_Channel_1, TIM2_IRQn,               0, IOCFG_AF_PP, }, // PPM/SERIAL RX
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, TIM3_IRQn,               0, IOCFG_AF_PP, }, // PWM1
    { TIM3,  IO_TAG(PB5),  TIM_Channel_2, TIM3_IRQn,               0, IOCFG_AF_PP, }, // PWM2
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               0, IOCFG_AF_PP, }, // PWM3
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               0, IOCFG_AF_PP, }, // PWM4
    { TIM16, IO_TAG(PB8),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, }, // PWM5
    { TIM17, IO_TAG(PB9),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP, }, // PWM6
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, }, // SOFTSERIAL1 RX (NC)
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, }, // SOFTSERIAL1 TX
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP, }, // LED_STRIP
};

