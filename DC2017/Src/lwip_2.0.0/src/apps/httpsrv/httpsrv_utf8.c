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
/*
*   This file contains various UTF-8 functions.
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "httpsrv_utf8.h"

static inline bool utf8_check_boundary(uint8_t *position, uint32_t n, uint8_t *max);

/*
 * Check if input sequence is valid UTF-8 sequence.
 */
bool utf8_is_valid(uint8_t *input, uint32_t length, uint8_t **bad, uint32_t *missing)
{
    uint8_t *position;
    uint8_t *max;
    uint32_t n;

    position = input;
    max = input + length;
    *missing = 0;

    while (position < max)
    {
        n = 0;

        if (utf8_check_1(position))
        {
            position++;
            continue;
        }
        n++;
        if (utf8_check_boundary(position, n, max))
        {
            *missing = 4 - n;
            goto BOUNDARY;
        }

        if (utf8_check_2(position))
        {
            position += n + 1;
            continue;
        }
        n++;
        if (utf8_check_boundary(position, n, max))
        {
            *missing = 4 - n;
            goto BOUNDARY;
        }
        if (utf8_check_3(position))
        {
            position += n + 1;
            continue;
        }
        n++;
        if (utf8_check_boundary(position, n, max))
        {
            *missing = 4 - n;
            goto BOUNDARY;
        }
        if (utf8_check_4(position))
        {
            position += n + 1;
            continue;
        }

    BOUNDARY:
        *bad = position;
        return (false);
    }

    *bad = NULL;
    return (true);
}

/* Check if next step would break array boundary. */
static inline bool utf8_check_boundary(uint8_t *position, uint32_t n, uint8_t *max)
{
    if ((position + n) >= max)
    {
        return (true);
    }
    return (false);
}
