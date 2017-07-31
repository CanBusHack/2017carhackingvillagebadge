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
/* *   This file contains public definitions for websocket protocol */
#ifndef __httpsrv_ws_h__
#define __httpsrv_ws_h__

#include "httpsrv.h"

#if HTTPSRV_CFG_WEBSOCKET_ENABLED

/*
 * WebSocket data type
 */
typedef enum ws_data_type
{
    WS_DATA_INVALID,
    WS_DATA_TEXT,
    WS_DATA_BINARY
} WS_DATA_TYPE;

/*
 * WebSocket errors
 */
typedef enum ws_error_code
{
    WS_ERR_OK,
    WS_ERR_SOCKET,
    WS_ERR_BAD_FRAME,
    WS_ERR_BAD_OPCODE,
    WS_ERR_BAD_SEQ,
    WS_ERR_NO_UTF8,
    WS_ERR_SERVER
} WS_ERROR_CODE;

/*
 * WebSocket data structure
 */
typedef struct ws_data_struct
{
    /* Pointer to user data. */
    uint8_t *data_ptr;
    /* Length of user data. */
    uint32_t length;
    /* Type of data. */
    WS_DATA_TYPE type;
} WS_DATA_STRUCT;

/*
 * Structure passed as parameter to user callbacks
 */
typedef struct ws_user_context_struct
{
    /* WebSocket handle. */
    uint32_t handle;
    /* Error code if error occurs. */
    WS_ERROR_CODE error;
    /* Data structure. */
    WS_DATA_STRUCT data;
    /* Flag signalizing end of message. */
    uint32_t fin_flag;
} WS_USER_CONTEXT_STRUCT;

/*
 * WebSocket callback function prototype.
 */
typedef uint32_t (*WS_CALLBACK_FN)(void *param, WS_USER_CONTEXT_STRUCT context);

/*
 * Structure defining WebSocket plugin.
 *
 * Usually one resource on server is mapped to one plugin.
 */
typedef struct ws_plugin_struct
{
    /* Path of resource causing plugin invocation. */
    char *resource;
    /* on_connect - when client connects to server. */
    WS_CALLBACK_FN on_connect;
    /* on_message - when message is received from client. */
    WS_CALLBACK_FN on_message;
    /* on_error - when error occurs. */
    WS_CALLBACK_FN on_error;
    /* on_disconnect - when client disconnects from server.*/
    WS_CALLBACK_FN on_disconnect;
    /* callback parameter(s) */
    void *cookie;
} WS_PLUGIN_STRUCT;

#ifdef __cplusplus
extern "C" {
#endif

int32_t WS_send(WS_USER_CONTEXT_STRUCT *context);
int32_t WS_close(uint32_t handle);

#ifdef __cplusplus
extern "C" {
#endif

#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */

#endif
