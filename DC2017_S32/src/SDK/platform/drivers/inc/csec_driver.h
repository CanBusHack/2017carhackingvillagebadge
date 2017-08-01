/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CSEC_DRV_H
#define CSEC_DRV_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "device_registers.h"
#include "csec_hal.h"
#include "interrupt_manager.h"

/*! @file */

/*!
 * @defgroup csec_driver CSEc Driver
 * @ingroup csec
 * @brief Cryptographic Services Engine Peripheral Driver.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Specifies the boot type for the BOOT_DEFINE command.
 *
 * Implements : csec_boot_flavor_t_Class
 */
typedef enum {
    CSEC_BOOT_STRICT,
    CSEC_BOOT_SERIAL,
    CSEC_BOOT_PARALLEL,
    CSEC_BOOT_NOT_DEFINED
} csec_boot_flavor_t;

/*!
 * @brief CSEc asynchronous command complete callback function type.
 *
 * Implements : csec_callback_t_Class
 */
typedef void (*csec_callback_t)(csec_cmd_t completedCmd, void *callbackParam);

/*!
 * @brief Internal driver state information.
 *
 * @note The contents of this structure are internal to the driver and should not be
 *      modified by users. Also, contents of the structure are subject to change in
 *      future releases.
 *
 * Implements : csec_state_t_Class
 */
typedef struct {
    bool cmdInProgress;           /*!< Specifies if a command is in progress */
    csec_cmd_t cmd;               /*!< Specifies the type of the command in execution */
    const uint8_t *inputBuff;     /*!< Specifies the input of the command in execution */
    uint8_t *outputBuff;          /*!< Specifies the output of the command in execution */
    uint32_t index;               /*!< Specifies the index in the input buffer of the command in execution */
    uint32_t fullSize;            /*!< Specifies the size of the input of the command in execution */
    uint32_t partSize;            /*!< Specifies the size of the chunck of the input currently processed */
    csec_key_id_t keyId;          /*!< Specifies the key used for the command in execution */
    status_t errCode;    /*!< Specifies the error code of the last executed command */
    const uint8_t *iv;            /*!< Specifies the IV of the command in execution (for encryption/decryption using CBC mode) */
    csec_call_sequence_t seq;     /*!< Specifies if the information is the first or a following function call. */
    uint32_t msgLen;              /*!< Specifies the message size (in bits) for the command in execution (for MAC generation/verification) */
    bool *verifStatus;            /*!< Specifies the result of the last executed MAC verification command */
    bool macWritten;              /*!< Specifies if the MAC to be verified was written in CSE_PRAM for a MAC verification command */
    const uint8_t *mac;           /*!< Specifies the MAC to be verified for a MAC verification command */
    uint32_t macLen;              /*!< Specifies the number of bits of the MAC to be verified for a MAC verification command */
    csec_callback_t callback;     /*!< The callback invoked when an asynchronous command is completed */
    void *callbackParam;          /*!< User parameter for the command completion callback */
} csec_state_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the internal state of the driver and enables the FTFC
 * interrupt.
 *
 * @param[in] state Pointer to the state structure which will be used for holding
 * the internal state of the driver.
 */
void CSEC_DRV_Init(csec_state_t *state);

/*!
 * @brief Clears the internal state of the driver and disables the FTFC
 * interrupt.
 */
void CSEC_DRV_Deinit(void);

/*!
 * @brief Performs the AES-128 encryption in ECB mode.
 *
 * This function performs the AES-128 encryption in ECB mode of the input
 * plain text buffer
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * It should be multiple of 16 bytes.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_EncryptECB(csec_key_id_t keyId,
    const uint8_t *plainText, uint32_t length, uint8_t *cipherText);

/*!
 * @brief Performs the AES-128 decryption in ECB mode.
 *
 * This function performs the AES-128 decryption in ECB mode of the input
 * cipher text buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation
 * @param[in] cipherText Pointer to the cipher text buffer.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_DecryptECB(csec_key_id_t keyId, const uint8_t *cipherText,
    uint32_t length, uint8_t *plainText);

/*!
 * @brief Performs the AES-128 encryption in CBC mode.
 *
 * This function performs the AES-128 encryption in CBC mode of the input
 * plaintext buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_EncryptCBC(csec_key_id_t keyId,
    const uint8_t *plainText, uint32_t length,
    const uint8_t *iv, uint8_t *cipherText);

/*!
 * @brief Performs the AES-128 decryption in CBC mode.
 *
 * This function performs the AES-128 decryption in CBC mode of the input
 * cipher text buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] cipherText Pointer to the cipher text buffer.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_DecryptCBC(csec_key_id_t keyId, const uint8_t *cipherText,
    uint16_t length, const uint8_t* iv, uint8_t *plainText);

/*!
 * @brief Calculates the MAC of a given message using CMAC with AES-128.
 *
 * This function calculates the MAC of a given message using CMAC with AES-128.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] cmac Pointer to the buffer containing the result of the CMAC
 * computation.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_GenerateMAC(csec_key_id_t keyId, const uint8_t *msg,
    uint32_t msgLen, uint8_t *cmac);

/*!
 * @brief Calculates the MAC of a given message (located in Flash) using CMAC
 * with AES-128.
 *
 * This function calculates the MAC of a given message using CMAC with AES-128.
 * It is different from the CSEC_DRV_GenerateMAC function in the sense that it
 * does not involve an extra copy of the data on which the CMAC is computed and
 * the message pointer should be a pointer to Flash memory.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer (pointing to Flash memory).
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] cmac Pointer to the buffer containing the result of the CMAC
 * computation.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_GenerateMACAddrMode(csec_key_id_t keyId,
    const uint8_t *msg, uint32_t msgLen, uint8_t *cmac);

/*!
 * @brief Verifies the MAC of a given message using CMAC with AES-128.
 *
 * This function verifies the MAC of a given message using CMAC with AES-128.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the CMAC to be verified.
 * @param[in] macLen Number of bits of the CMAC to be compared. A macLength
 * value of zero indicates that all 128-bits are compared.
 * @param[out] verifStatus Status of MAC verification command (true:
 * verification operation passed, false: verification operation failed).
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_VerifyMAC(csec_key_id_t keyId, const uint8_t *msg,
    uint32_t msgLen, const uint8_t *mac, uint16_t macLen, bool *verifStatus);

/*!
 * @brief Verifies the MAC of a given message (located in Flash) using CMAC with
 * AES-128.
 *
 * This function verifies the MAC of a given message using CMAC with AES-128.
 * It is different from the CSEC_DRV_VerifyMAC function in the sense that it
 * does not involve an extra copy of the data on which the CMAC is computed and
 * the message pointer should be a pointer to Flash memory.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer (pointing to Flash memory).
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the CMAC to be verified.
 * @param[in] macLen Number of bits of the CMAC to be compared. A macLength
 * value of zero indicates that all 128-bits are compared.
 * @param[out] verifStatus Status of MAC verification command (true:
 * verification operation passed, false: verification operation failed).
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_VerifyMACAddrMode(csec_key_id_t keyId, const uint8_t *msg,
    uint32_t msgLen, const uint8_t *mac, uint16_t macLen, bool *verifStatus);

/*!
 * @brief Updates an internal key per the SHE specification.
 *
 * This function updates an internal key per the SHE specification.
 *
 * @param[in] keyId KeyID of the key to be updated.
 * @param[in] m1 Pointer to the 128-bit M1 message containing the UID, Key ID
 * and Authentication Key ID.
 * @param[in] m2 Pointer to the 256-bit M2 message contains the new security
 * flags, counter and the key value all encrypted using a derived key generated
 * from the Authentication Key.
 * @param[in] m3 Pointer to the 128-bit M3 message is a MAC generated over
 * messages M1 and M2.
 * @param[out] m4 Pointer to a 256 bits buffer where the computed M4 parameter
 * is stored.
 * @param[out] m5 Pointer to a 128 bits buffer where the computed M5 parameters
 * is stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_LoadKey(csec_key_id_t keyId, const uint8_t *m1,
    const uint8_t *m2, const uint8_t *m3, uint8_t *m4, uint8_t *m5);

/*!
 * @brief Updates the RAM key memory slot with a 128-bit plaintext.
 *
 * The function updates the RAM key memory slot with a 128-bit plaintext. The
 * key is loaded without encryption and verification of the key, i.e. the key is
 * handed over in plaintext. A plain key can only be loaded into the RAM_KEY
 * slot.
 *
 * @param[in] plainKey Pointer to the 128-bit buffer containing the key that
 * needs to be copied in RAM_KEY slot.
 * @return Error Code after command execution.
 */
status_t CSEC_DRV_LoadPlainKey(const uint8_t *plainKey);

/*!
 * @brief Exports the RAM_KEY into a format protected by SECRET_KEY.
 *
 * This function exports the RAM_KEY into a format protected by SECRET_KEY.
 *
 * @param[out] m1 Pointer to a buffer where the M1 parameter will be exported.
 * @param[out] m2 Pointer to a buffer where the M2 parameter will be exported.
 * @param[out] m3 Pointer to a buffer where the M3 parameter will be exported.
 * @param[out] m4 Pointer to a buffer where the M4 parameter will be exported.
 * @param[out] m5 Pointer to a buffer where the M5 parameter will be exported.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_ExportRAMKey(uint8_t *m1, uint8_t *m2,
    uint8_t *m3, uint8_t *m4, uint8_t *m5);

/*!
 * @brief Initializes the seed and derives a key for the PRNG.
 *
 * The function initializes the seed and derives a key for the PRNG.
 * The function must be called before CMD_RND after every power cycle/reset.
 *
 * @return Error Code after command execution.
 */
status_t CSEC_DRV_InitRNG(void);

/*!
 * @brief Extends the seed of the PRNG.
 *
 * Extends the seed of the PRNG by compressing the former seed value and the
 * supplied entropy into a new seed. This new seed is then to be used to
 * generate a random number by invoking the CMD_RND command. The random number
 * generator must be initialized by CMD_INIT_RNG before the seed may be
 * extended.
 *
 * @param[in] entropy Pointer to a 128-bit buffer containing the entropy.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_ExtendSeed(const uint8_t *entropy);

/*!
 * @brief Generates a vector of 128 random bits.
 *
 * The function returns a vector of 128 random bits. The random number generator
 * has to be initialized by calling CSEC_DRV_InitRNG before random numbers can
 * be supplied.
 *
 * @param[out] rnd Pointer to a 128-bit buffer where the generated random number
 * has to be stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_GenerateRND(uint8_t *rnd);

/*!
 * @brief Signals a failure detected during later stages of the boot process.
 *
 * The function is called during later stages of the boot process to detect a
 * failure.
 *
 * @return Error Code after command execution.
 */
status_t CSEC_DRV_BootFailure(void);

/*!
 * @brief Marks a successful boot verification during later stages of the boot
 * process.
 *
 * The function is called during later stages of the boot process to mark
 * successful boot verification.
 *
 * @return Error Code after command execution.
 */
status_t CSEC_DRV_BootOK(void);

/*!
 * @brief Implements an extension of the SHE standard to define both the user
 * boot size and boot method.
 *
 * The function implements an extension of the SHE standard to define both the
 * user boot size and boot method.
 *
 * @param[in] bootSize Number of blocks of 128-bit data to check on boot.
 * Maximum size is 512kBytes.
 * @param[in] bootFlavor The boot method.
 * @return Error Code after command execution.
 */
status_t CSEC_DRV_BootDefine(uint32_t bootSize, csec_boot_flavor_t bootFlavor);

/*!
 * @brief Returns the content of the status register.
 *
 * The function shall return the content of the status register.
 *
 * @return Value of the status register.
 *
 * Implements : CSEC_DRV_GetStatus_Activity
 */
static inline csec_status_t CSEC_DRV_GetStatus(void)
{
    return CSEC_HAL_ReadStatus();
}

/*!
 * @brief Returns the identity (UID) and the value of the status register
 * protected by a MAC over a challenge and the data.
 *
 * This function returns the identity (UID) and the value of the status register
 * protected by a MAC over a challenge and the data.
 *
 * @param[in] challenge Pointer to the 128-bit buffer containing Challenge data.
 * @param[out] uid Pointer to 120 bit buffer where the UID will be stored.
 * @param[out] sreg Value of the status register.
 * @param[out] mac Pointer to the 128 bit buffer where the MAC generated over
 * challenge and UID and status  will be stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_GetID(const uint8_t *challenge, uint8_t *uid,
    uint8_t *sreg, uint8_t *mac);

/*!
 * @brief Obtains a random number which the user shall use along with the
 * MASTER_ECU_KEY and UID to return an authorization request.
 *
 * This function obtains a random number which the user shall use along with the
 * MASTER_ECU_KEY and UID to return an authorization request.
 *
 * @param[out] challenge Pointer to the 128-bit buffer where the challenge data
 * will be stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_DbgChal(uint8_t *challenge);

/*!
 * @brief Erases all keys (actual and outdated) stored in NVM Memory if the
 * authorization is confirmed by CSEc.
 *
 * This function erases all keys (actual and outdated) stored in NVM Memory if
 * the authorization is confirmed by CSEc.
 *
 * @param[in] authorization Pointer to the 128-bit buffer containing the
 * authorization value.
 * @return Error Code after command execution.
 */
status_t CSEC_DRV_DbgAuth(const uint8_t *authorization);

/*!
 * @brief Compresses the given messages by accessing  the Miyaguchi-Prenell
 * compression feature with in the CSEc feature set.
 *
 * This function accesses a Miyaguchi-Prenell compression feature within the
 * CSEc feature set to compress the given messages.
 *
 * @param[in] msg Pointer to the messages to be compressed. Messages must be
 * pre-processed per SHE specification if they do not already meet the full
 * 128-bit block size requirement.
 * @param[in] msgLen The number of 128 bit messages to be compressed.
 * @param[out] mpCompress Pointer to the 128 bit buffer storing the compressed
 * data.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSEC_DRV_MPCompress(const uint8_t *msg, uint16_t msgLen,
    uint8_t *mpCompress);

/*!
 * @brief Asynchronously performs the AES-128 encryption in ECB mode.
 *
 * This function performs the AES-128 encryption in ECB mode of the input
 * plain text buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * It should be multiple of 16 bytes.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 * @return STATUS_SUCCESS if the command was successfully launched, STATUS_BUSY if
 * another command was already launched. CSEC_DRV_GetAsyncCmdStatus can be used
 * in order to check the execution status.
 */
status_t CSEC_DRV_EncryptECBAsync(csec_key_id_t keyId,
                                           const uint8_t *plainText,
                                           uint32_t length,
                                           uint8_t *cipherText);

/*!
 * @brief Asynchronously performs the AES-128 decryption in ECB mode.
 *
 * This function performs the AES-128 decryption in ECB mode of the input
 * cipher text buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation
 * @param[in] cipherText Pointer to the cipher text buffer.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 * @return STATUS_SUCCESS if the command was successfully launched, STATUS_BUSY if
 * another command was already launched. CSEC_DRV_GetAsyncCmdStatus can be used
 * in order to check the execution status.
 */
status_t CSEC_DRV_DecryptECBAsync(csec_key_id_t keyId,
                                           const uint8_t *cipherText,
                                           uint32_t length,
                                           uint8_t *plainText);
/*!
 * @brief Asynchronously performs the AES-128 encryption in CBC mode.
 *
 * This function performs the AES-128 encryption in CBC mode of the input
 * plaintext buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 * @return STATUS_SUCCESS if the command was successfully launched, STATUS_BUSY if
 * another command was already launched. CSEC_DRV_GetAsyncCmdStatus can be used
 * in order to check the execution status.
 */
status_t CSEC_DRV_EncryptCBCAsync(csec_key_id_t keyId,
                                           const uint8_t *cipherText,
                                           uint16_t length,
                                           const uint8_t* iv,
                                           uint8_t *plainText);

/*!
 * @brief Asynchronously performs the AES-128 decryption in CBC mode.
 *
 * This function performs the AES-128 decryption in CBC mode of the input
 * cipher text buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] cipherText Pointer to the cipher text buffer.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 * @return STATUS_SUCCESS if the command was successfully launched, STATUS_BUSY if
 * another command was already launched. CSEC_DRV_GetAsyncCmdStatus can be used
 * in order to check the execution status.
 */
status_t CSEC_DRV_DecryptCBCAsync(csec_key_id_t keyId,
                                           const uint8_t *cipherText,
                                           uint32_t length,
                                           const uint8_t* iv,
                                           uint8_t *plainText);

/*!
 * @brief Asynchronously calculates the MAC of a given message using CMAC with
 * AES-128.
 *
 * This function calculates the MAC of a given message using CMAC with AES-128,
 * in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] cmac Pointer to the buffer containing the result of the CMAC
 * computation.
 * @return STATUS_SUCCESS if the command was successfully launched, STATUS_BUSY if
 * another command was already launched. CSEC_DRV_GetAsyncCmdStatus can be used
 * in order to check the execution status.
 */
status_t CSEC_DRV_GenerateMACAsync(csec_key_id_t keyId,
                                            const uint8_t *msg,
                                            uint32_t msgLen,
                                            uint8_t *cmac);

/*!
 * @brief Asynchronously verifies the MAC of a given message using CMAC with
 * AES-128.
 *
 * This function verifies the MAC of a given message using CMAC with AES-128,
 * in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the CMAC to be verified.
 * @param[in] macLen Number of bits of the CMAC to be compared. A macLength
 * value of zero indicates that all 128-bits are compared.
 * @param[out] verifStatus Status of MAC verification command (true:
 * verification operation passed, false: verification operation failed).
 * @return STATUS_SUCCESS if the command was successfully launched, STATUS_BUSY if
 * another command was already launched. CSEC_DRV_GetAsyncCmdStatus can be used
 * in order to check the execution status.
 */
status_t CSEC_DRV_VerifyMACAsync(csec_key_id_t keyId,
                                          const uint8_t *msg,
                                          uint32_t msgLen,
                                          const uint8_t *mac,
                                          uint16_t macLen,
                                          bool *verifStatus);

/*!
 * @brief Checks the status of the execution of an asynchronous command.
 *
 * This function checks the status of the execution of an asynchronous command.
 * If the command is still in progress, returns STATUS_BUSY.
 *
 * @return Error Code after command execution.
 */
status_t CSEC_DRV_GetAsyncCmdStatus(void);

/*!
 * @brief Installs a callback function which will be invoked when an asynchronous
 * command finishes its execution.
 *
 * @param[in] callbackFunc The function to be invoked.
 * @param[in] callbackParam The parameter to be passed to the callback function.
 */
void CSEC_DRV_InstallCallback(csec_callback_t callbackFunc, void *callbackParam);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* CSEC_DRV_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
