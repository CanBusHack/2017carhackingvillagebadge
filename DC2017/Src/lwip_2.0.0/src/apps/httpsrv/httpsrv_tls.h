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

/*   Header for HTTPSRV TLS.*/

#ifndef _HTTPSRV_TLS_H_
#define _HTTPSRV_TLS_H_

#include "httpsrv_config.h"

#if HTTPSRV_CFG_WOLFSSL_ENABLE || HTTPSRV_CFG_MBEDTLS_ENABLE
#include "httpsrv.h"

#if HTTPSRV_CFG_WOLFSSL_ENABLE 
#include "wolfssl/ssl.h"

typedef WOLFSSL          *httpsrv_tls_sock_t;
typedef WOLFSSL_CTX      *httpsrv_tls_ctx_t;
#endif

#if HTTPSRV_CFG_MBEDTLS_ENABLE
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/certs.h"
#include "mbedtls/x509.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ssl_cache.h"
#include "mbedtls/debug.h"

typedef mbedtls_ssl_context *  httpsrv_tls_sock_t;

typedef struct {
                    mbedtls_entropy_context entropy;
                    mbedtls_ctr_drbg_context ctr_drbg;
                    mbedtls_ssl_config conf;
                    mbedtls_x509_crt srvcert;
                    mbedtls_pk_context pkey;
                #if defined(MBEDTLS_SSL_CACHE_C)
                    mbedtls_ssl_cache_context cache;
                #endif
                } *httpsrv_tls_ctx_t;

#endif

#ifdef __cplusplus
extern "C" {
#endif

httpsrv_tls_ctx_t httpsrv_tls_init(const HTTPSRV_TLS_PARAM_STRUCT *params);
void httpsrv_tls_release(httpsrv_tls_ctx_t ctx);
httpsrv_tls_sock_t httpsrv_tls_socket(httpsrv_tls_ctx_t ctx, int sock);
void httpsrv_tls_shutdown(httpsrv_tls_sock_t tls_sock);
int httpsrv_tls_recv(httpsrv_tls_sock_t tls_sock, void *buf, size_t len, int flags);
int32_t httpsrv_tls_send(httpsrv_tls_sock_t tls_sock, const void *buf, size_t len, int flags);

#ifdef __cplusplus
}
#endif

#endif /* HTTPSRV_CFG_WOLFSSL_ENABLE || HTTPSRV_CFG_MBEDTLS_ENABLE */

#endif /* _HTTPSRV_TLS_H_ */
