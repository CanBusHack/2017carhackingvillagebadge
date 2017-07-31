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
/*   This file contains API functions for WebSocket protocol.*/

#include "httpsrv.h"
#include "httpsrv_ws_prv.h"
#include "httpsrv_port.h"
#include "lwip/sys.h"


#if HTTPSRV_CFG_WEBSOCKET_ENABLED

/*
 * Send data through WebSocket.
 */
int32_t WS_send(WS_USER_CONTEXT_STRUCT* context)
{
    WS_CONTEXT_STRUCT *ws_context;
    WS_FRAME_STRUCT   *frame;
    WS_DATA_STRUCT    *data;
    WS_API_CALL_MSG     *message;

    ws_context = (WS_CONTEXT_STRUCT*) context->handle;
    data = &(context->data);

    /* Check input validity. */
    if ((data->type == WS_DATA_INVALID) ||
        (data->data_ptr == NULL) ||
        (ws_context == NULL))
    {
        return(HTTPSRV_ERR);
    }

    message = (WS_API_CALL_MSG *)httpsrv_mem_alloc(sizeof(WS_API_CALL_MSG));
    if (message == 0)
    {
        return HTTPSRV_ERR;
    }
    
    memset(message, 0, sizeof(WS_API_CALL_MSG));
    frame = &message->frame;
    /* Fill frame structure and send it */
    frame->opcode = data->type;
    frame->length = data->length;
    frame->data = data->data_ptr;
    frame->fin = (bool) context->fin_flag;

    message->command = WS_COMMAND_SEND;

    sys_mbox_post(&ws_context->api_queue, message);

    return(HTTPSRV_OK);
}

/*
 * Close WebSocket.
 */
int32_t WS_close(uint32_t handle)
{
    WS_CONTEXT_STRUCT *ws_context;
    WS_API_CALL_MSG   *message;
    int32_t           retval;

    ws_context = (WS_CONTEXT_STRUCT*) handle;

    retval = HTTPSRV_ERR;

    if (ws_context != NULL)
    {

        message = (WS_API_CALL_MSG *)httpsrv_mem_alloc(sizeof(WS_API_CALL_MSG));
        if (message == 0)
        {
            return HTTPSRV_ERR;
        }

        message->command = WS_COMMAND_CLOSE;

        sys_mbox_post(&ws_context->api_queue, message);
        
        retval = HTTPSRV_OK;
    }
    return(retval);
}

#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */
