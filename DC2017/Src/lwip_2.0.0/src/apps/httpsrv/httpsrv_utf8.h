/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __utf8_h__
#define __utf8_h__

#include <stdint.h>
#include <stdbool.h>

#define UTF8_TAIL_MIN 0x80
#define UTF8_TAIL_MAX 0xBF

#ifdef __cplusplus
extern "C" {
#endif

bool utf8_is_valid(uint8_t *input, uint32_t length, uint8_t **bad, uint32_t *missing);

static inline bool utf8_check_1(uint8_t *data);
static inline bool utf8_check_2(uint8_t *data);
static inline bool utf8_check_3(uint8_t *data);
static inline bool utf8_check_4(uint8_t *data);

/* Check 1 byte for validity. */
static inline bool utf8_check_1(uint8_t *data)
{
    if ((data[0] == 0x09) || (data[0] == 0x0A) || (data[0] == 0x0D) || (data[0] <= 0x7F))
    {
        return (true);
    }
    return (false);
}

/* Check 2 bytes for validity. */
static inline bool utf8_check_2(uint8_t *data)
{
    if (((data[0] >= 0xC2) && (data[0] <= 0xDF)) && ((data[1] >= 0x80) && (data[1] <= 0xBF)))
    {
        return (true);
    }
    return (false);
}

/* Check 3 bytes for validity. */
static inline bool utf8_check_3(uint8_t *data)
{
    if ((/* Excluding overlongs. */
         (data[0] == 0xE0) && ((data[1] >= 0xA0) && (data[1] <= 0xBF)) && ((data[2] >= 0x80) && (data[2] <= 0xBF))) ||
        (/* Straight 3-byte. */
         (((data[0] >= 0xE1) && (data[0] <= 0xEC)) || (data[0] == 0xEE) || (data[0] == 0xEF)) &&
         ((data[1] >= 0x80) && (data[1] <= 0xBF)) && ((data[2] >= 0x80) && (data[2] <= 0xBF))) ||
        (/* Excluding surrogates. */
         (data[0] == 0xED) && ((data[1] >= 0x80) && (data[1] <= 0x9F)) && ((data[2] >= 0x80) && (data[2] <= 0xBF))))
    {
        return (true);
    }
    return (false);
}

/* Check 4 bytes for validity. */
static inline bool utf8_check_4(uint8_t *data)
{
    if ((/* Planes 1-3. */
         (data[0] == 0xF0) && ((data[1] >= 0x90) && (data[1] <= 0xBF)) && ((data[2] >= 0x80) && (data[2] <= 0xBF)) &&
         ((data[3] >= 0x80) && (data[3] <= 0xBF))) ||
        (/* Planes 4-15. */
         ((data[0] >= 0xF1) && (data[0] <= 0xF3)) && ((data[1] >= 0x80) && (data[1] <= 0xBF)) &&
         ((data[2] >= 0x80) && (data[2] <= 0xBF)) && ((data[3] >= 0x80) && (data[3] <= 0xBF))) ||
        (/* Plane 16. */
         (data[0] == 0xF4) && ((data[1] >= 0x80) && (data[1] <= 0x8F)) && ((data[2] >= 0x80) && (data[2] <= 0xBF)) &&
         ((data[3] >= 0x80) && (data[3] <= 0xBF))))
    {
        return (true);
    }
    return (false);
}

#ifdef __cplusplus
}
#endif

#endif
