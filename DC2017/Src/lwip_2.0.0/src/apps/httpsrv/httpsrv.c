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

/*   This file contains the HTTPSRV implementation.*/

#include "httpsrv.h"
#include "httpsrv_supp.h"
#include "httpsrv_prv.h"
#include <string.h>

#define HTTPSRV_SERVER_TASK_NAME "HTTP server"

/*
** Function for starting the HTTP server
**
** IN:
**      HTTPSRV_PARAM_STRUCT*   params - server parameters (port, ip, index page etc.)
**
** OUT:
**      none
**
** Return Value:
**      uint32_t      server handle if successful, NULL otherwise
*/
uint32_t HTTPSRV_init(HTTPSRV_PARAM_STRUCT *params)
{
    HTTPSRV_STRUCT *server;
    uint32_t handle = 0;

    server = httpsrv_create_server(params);
    if (server)
    {
        server->valid = HTTPSRV_VALID;
        server->server_tid = sys_thread_new(HTTPSRV_SERVER_TASK_NAME, httpsrv_server_task, server,
                                            HTTPSRV_CFG_SERVER_STACK_SIZE, server->params.task_prio);
        if (server->server_tid == NULL)
        {
            httpsrv_destroy_server(server);
            httpsrv_mem_free(server);
        }
        else
        {
            handle = (uint32_t)server;
        }
    }

    return (handle);
}

/*
** Function for releasing/stopping HTTP server
**
** IN:
**      uint32_t       server_h - server handle
**
** OUT:
**      none
**
** Return Value:
**      uint32_t      error code. HTTPSRV_OK if everything went right, positive number otherwise
*/
void HTTPSRV_release(uint32_t server_h)
{
    HTTPSRV_STRUCT *server = (void *)server_h;

    if (server)
    {
        /* Shutdown server task and wait for its termination. */
        if (server->sock != -1)
        {
            server->valid = HTTPSRV_INVALID;
            lwip_close(server->sock); /* Ublock select() */
        }

        while (server->server_tid) /* Wait for task completition.*/
        {
            taskYIELD();
        }
        httpsrv_mem_free(server);
    }
}

/*
** Write data to client from CGI script
**
** IN:
**      HTTPSRV_CGI_RES_STRUCT* response - CGI response structure used for forming response
**
** OUT:
**      none
**
** Return Value:
**      uint_32 - Number of bytes written
*/
uint32_t HTTPSRV_cgi_write(HTTPSRV_CGI_RES_STRUCT *response)
{
    HTTPSRV_SESSION_STRUCT *session = (HTTPSRV_SESSION_STRUCT *)response->ses_handle;
    uint32_t retval = 0;
    int32_t wrote;

    if (session == NULL)
    {
        return (0);
    }

    if (!(session->flags & HTTPSRV_FLAG_HEADER_SENT))
    {
        session->response.status_code = response->status_code;
        session->response.content_type = response->content_type;
        session->response.length = response->content_length;

        if (response->content_length < 0)
        {
            session->flags |= HTTPSRV_FLAG_IS_TRANSCODED;
        }

        /*
         * Ignore rest of received data in buffer (set buffer offset to zero).
         * We do this because otherwise any buffered but unread data would be
         * part of response (before header) and render it invalid.
         */
        if (session->buffer.offset <= session->request.content_length)
        {
            session->request.content_length -= session->buffer.offset;
        }
        session->buffer.offset = 0;
        httpsrv_sendhdr(session, response->content_length, 1);
    }

    if (session->flags & HTTPSRV_FLAG_IS_TRANSCODED)
    {
        char length_str[sizeof("FFFFFFFF\r\n")] = {0};

        /* Write length. */
        snprintf(length_str, sizeof(length_str), "%x\r\n", (unsigned int)response->data_length);
        wrote = httpsrv_write(session, length_str, strlen(length_str));
        if (wrote < 0)
        {
            retval = 0;
            goto EXIT;
        }
        /* Write data. */
        wrote = httpsrv_write(session, response->data, response->data_length);
        if (wrote < 0)
        {
            retval = 0;
            goto EXIT;
        }
        retval = wrote;
        /* Write tail. */
        wrote = httpsrv_write(session, "\r\n", strlen("\r\n"));
        if (wrote < 0)
        {
            retval = 0;
            goto EXIT;
        }
    }
    else if ((response->data != NULL) && (response->data_length > 0))
    {
        retval = httpsrv_write(session, response->data, response->data_length);
    }

EXIT:
    session->time = sys_now();
    return (retval);
}

/*
** Read data from client to CGI script
**
** IN:
**      uint32_t ses_handle - handle to session used for reading
**      char*   buffer - user buffer to read data to
**      uint32_t length - size of buffer in bytes
**
** OUT:
**      none
**
** Return Value:
**      uint32_t - Number of bytes read
*/
uint32_t HTTPSRV_cgi_read(uint32_t ses_handle, char *buffer, uint32_t length)
{
    HTTPSRV_SESSION_STRUCT *session = (HTTPSRV_SESSION_STRUCT *)ses_handle;
    uint32_t retval;

    if ((session == NULL) || (buffer == NULL) || (length == 0))
    {
        return (0);
    }

    retval = httpsrv_read(session, buffer, length);

    if (retval > 0)
    {
        if (retval <= session->request.content_length)
        {
            session->request.content_length -= retval;
        }
    }
    else
    {
        int optval;
        socklen_t optlen = sizeof(optval);

        /* Timeout */
        if ((lwip_getsockopt(session->sock, SOL_SOCKET, SO_ERROR, &optval, &optlen) == 0) && (optval == EWOULDBLOCK))
        {
            retval = 0;
        }
    }
    session->time = sys_now();
    return (retval);
}

/*
** Write data to client from server side include
**
** IN:
**      uint32_t ses_handle - session foe writing
**      char*   data - user data to write
**      uint32_t length - size of data in bytes
**
** OUT:
**      none
**
** Return Value:
**      uint32_t - Number of bytes written
*/
uint32_t HTTPSRV_ssi_write(uint32_t ses_handle, char *data, uint32_t length)
{
    HTTPSRV_SESSION_STRUCT *session = (HTTPSRV_SESSION_STRUCT *)ses_handle;
    uint32_t retval = 0;

    if ((session != NULL) && (data != NULL) && (length))
    {
        retval = httpsrv_write(session, data, length);
    }

    return (retval);
}
