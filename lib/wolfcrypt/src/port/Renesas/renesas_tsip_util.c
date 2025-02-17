/* renesas_tsip_util.c
 *
 * Copyright (C) 2006-2020 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */
#include <wolfcrypt_settings.h>

#if defined(WOLFSSL_RENESAS_TSIP)

#include <wc_port.h>
#include <error-crypt.h>

#include <port/Renesas/renesas-tsip-crypt.h>
#include <wolfcrypt_memory.h>
#include <error-crypt.h>
#include <aes.h>
#include <ssl.h>
#include <internal.h>

#include <stdio.h>




/* ./ca-cert.der.sign,  */
/* expect to have these variables defined at user application */
extern uint32_t     s_flash[];
extern uint32_t     s_inst1[R_TSIP_SINST_WORD_SIZE];
extern uint32_t     s_inst2[R_TSIP_SINST2_WORD_SIZE];


wolfSSL_Mutex       tsip_mutex;
static int          tsip_CryptHwMutexInit_ = 0;
static const byte*  ca_cert_sig;
static tsip_key_data g_user_key_info;

/* tsip only keep one encrypted ca public key */
#if defined(WOLFSSL_RENESAS_TSIP_TLS)
static uint32_t     g_encrypted_publicCA_key[R_TSIP_SINST_WORD_SIZE];
static uint32_t     g_CAscm_Idx;          /* index of CM table    */
#endif



static int tsip_CryptHwMutexInit(wolfSSL_Mutex* mutex) 
{
    return wc_InitMutex(mutex);
}

static int tsip_CryptHwMutexLock(wolfSSL_Mutex* mutex) 
{
    return wc_LockMutex(mutex);
}

static int tsip_CryptHwMutexUnLock(wolfSSL_Mutex* mutex)
{
    return wc_UnLockMutex(mutex);
}

#if defined(WOLFSSL_RENESAS_TSIP_TLS) && (WOLFSSL_RENESAS_TSIP_VER >=109)

static uint32_t GetTsipCipherSuite( 
                    uint8_t cipherSuiteFirst,
                    uint8_t cipherSuite)
{
    WOLFSSL_MSG(">> GetTsipCipherSuite");
    uint32_t tsipCipher;

    if(cipherSuiteFirst == CIPHER_BYTE )
    {
        switch(cipherSuite){

            case TLS_RSA_WITH_AES_128_CBC_SHA: /*2F*/
                tsipCipher = R_TSIP_TLS_RSA_WITH_AES_128_CBC_SHA; /*0*/
                break;

            case TLS_RSA_WITH_AES_128_CBC_SHA256:
                tsipCipher = R_TSIP_TLS_RSA_WITH_AES_128_CBC_SHA256;
                break;

            case TLS_RSA_WITH_AES_256_CBC_SHA:
                tsipCipher = R_TSIP_TLS_RSA_WITH_AES_256_CBC_SHA;
                break;

            case TLS_RSA_WITH_AES_256_CBC_SHA256:
                tsipCipher = R_TSIP_TLS_RSA_WITH_AES_256_CBC_SHA256;
                break;

            default:
                tsipCipher = (uint32_t)WOLFSSL_TSIP_ILLEGAL_CIPHERSUITE;
                break;
		}
        WOLFSSL_MSG( "<< GetTsipCipherSuite");
        return tsipCipher;
	}
    else if( cipherSuiteFirst == ECC_BYTE )
    {
        tsipCipher = (uint32_t)WOLFSSL_TSIP_ILLEGAL_CIPHERSUITE;
        /* comment out until implementation completes
        switch(cipherSuite){

            case TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256:
                tsipCipher = R_TSIP_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256;
                break;

            case TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256:
                tsipCipher = R_TSIP_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
                break;

            case TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256:
                tsipCipher = R_TSIP_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256;
                break;

            case TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256:
                tsipCipher = R_TSIP_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256;
                break;

            default:
                tsipCipher = (uint32_t)WOLFSSL_TSIP_ILLEGAL_CIPHERSUITE;
                break;
        } */
    }
	else{
        tsipCipher = (uint32_t)WOLFSSL_TSIP_ILLEGAL_CIPHERSUITE;
    }

    WOLFSSL_MSG( "<< GetTsipCipherSuite" );

    return tsipCipher;
}
#elif defined(WOLFSSL_RENESAS_TSIP_TLS) && (WOLFSSL_RENESAS_TSIP_VER >=106) 

/* convert def to tsip define */
static byte _tls2tsipdef(byte cipher)
{
    byte def = R_TSIP_TLS_RSA_WITH_AES_128_CBC_SHA;
    switch(cipher){
        case l_TLS_RSA_WITH_AES_128_CBC_SHA:
            break;
        case l_TLS_RSA_WITH_AES_128_CBC_SHA256:
            def = R_TSIP_TLS_RSA_WITH_AES_128_CBC_SHA256;
            break;
        case l_TLS_RSA_WITH_AES_256_CBC_SHA:
            def = R_TSIP_TLS_RSA_WITH_AES_256_CBC_SHA;
            break;
        case l_TLS_RSA_WITH_AES_256_CBC_SHA256:
            def = R_TSIP_TLS_RSA_WITH_AES_256_CBC_SHA256;
            break;
        default:break;
    }
    return def;
}
#endif
/*
* lock hw engine.
* this should be called before using engine.
*/
int tsip_hw_lock()
{
    int ret = 0;

    if(tsip_CryptHwMutexInit_ == 0){
      
        ret = tsip_CryptHwMutexInit(&tsip_mutex);
      
        if(ret == 0) {
            tsip_CryptHwMutexInit_ = 1;
        } else {
            WOLFSSL_MSG(" mutex initialization failed.");
            return -1;
        }
    }
    if(tsip_CryptHwMutexLock(&tsip_mutex) != 0) {
        /* this should not happens */
        return -1;
    }
 
    return ret;
}

/*
* release hw engine
*/
void tsip_hw_unlock( void )
{
    tsip_CryptHwMutexUnLock(&tsip_mutex);
}

/* check if tsip tls functions can be used for the cipher      */
/* cipher0 : in the some cipher suite,                         */
/*           first byte becomes greater than 0, otherwise 0x00 */
/* side    : CLIENT END or SEVER END                           */
int tsip_useable(const struct WOLFSSL *ssl)
{
    WOLFSSL_MSG(">> tsip_useable"); 
    byte cipher0;
    byte cipher;
    byte side;
    
    /* sanity check */
    if (ssl == NULL)
        return BAD_FUNC_ARG;
    
    /* when rsa key index == NULL, tsip isn't used for cert verification. */
    /* in the case, we cannot use TSIP.                                   */
    if (!ssl->peerTsipEncRsaKeyIndex)
        return 0;
    
    /* when enabled Extended Master Secret, we cannot use TSIP.           */
    if (ssl->options.haveEMS)
        return 0;
    
    cipher0 = ssl->options.cipherSuite0;
    cipher = ssl->options.cipherSuite;
    side = ssl->options.side;
    
    if (cipher0 > 0x00) 
        return 0;
    
    if ((cipher == l_TLS_RSA_WITH_AES_128_CBC_SHA ||
         cipher == l_TLS_RSA_WITH_AES_128_CBC_SHA256 ||
         cipher == l_TLS_RSA_WITH_AES_256_CBC_SHA ||
         cipher == l_TLS_RSA_WITH_AES_256_CBC_SHA256) &&
         side == WOLFSSL_CLIENT_END)
        return 1;
    else
        return 0;
}

/* check if the g_alreadyVerified CA's key can be used for *
 * peer's certification                                    */
byte tsip_checkCA(word32 cmIdx)
{
    return (cmIdx == g_CAscm_Idx? 1:0);
}

/* check if the root CA has been verified by TSIP, *
 * and it exists in the CM table.                  */
byte tsip_rootCAverified( )
{
    return (g_CAscm_Idx != (uint32_t)-1 ? 1:0);
}

/* open TSIP driver for use */
int tsip_Open()
{
    WOLFSSL_MSG(">> tsip_Open");    
    int ret;
    
    if ((ret = tsip_hw_lock()) == 0) {
    
#if defined(WOLFSSL_RENESAS_TSIP_TLS) && (WOLFSSL_RENESAS_TSIP_VER>=109)

    ret = R_TSIP_Open(NULL,NULL);
    if( ret != TSIP_SUCCESS ) {
        WOLFSSL_MSG("RENESAS TSIP Open failed");
    }

    if( ret == TSIP_SUCCESS && g_user_key_info.encrypted_user_tls_key ){

        ret = R_TSIP_GenerateTlsRsaPublicKeyIndex(
                g_user_key_info.encrypted_provisioning_key,
                g_user_key_info.iv,
                g_user_key_info.encrypted_user_tls_key,
                &g_user_key_info.user_rsa2048_tls_pubindex); /* OUT */

        R_TSIP_Close();       /* close once */

        if( ret != TSIP_SUCCESS ){            

            WOLFSSL_MSG("R_TSIP_GenerataeTlsRsa: NG" );

        } else {                
            
            /* open again with newly created TLS public key index*/
            ret = R_TSIP_Open(
                    &g_user_key_info.user_rsa2048_tls_pubindex,
                    (tsip_update_key_ring_t*)s_inst2);

            if (ret != TSIP_SUCCESS) {
                WOLFSSL_MSG("R_TSIP_(Re)Open: NG");
            }
                /* init vars */
            g_CAscm_Idx = (uint32_t)-1;
        }
    }

#elif defined(WOLFSSL_RENESAS_TSIP_TLS) && (WOLFSSL_RENESAS_TSIP_VER>=106)

        ret = R_TSIP_Open((uint32_t*)s_flash, s_inst1, s_inst2);
        if( ret != TSIP_SUCCESS ) {
            WOLFSSL_MSG("RENESAS TSIP Open failed");
        }
        
        /* generate TLS Rsa public key for Certificate verification */
        if (ret == TSIP_SUCCESS && g_user_key_info.encrypted_user_tls_key) {
            ret = R_TSIP_GenerateTlsRsaPublicKeyIndex(
                    g_user_key_info.encrypted_session_key,
                    g_user_key_info.iv,
                    g_user_key_info.encrypted_user_tls_key,
                    &g_user_key_info.user_rsa2048_tls_pubindex);
            
            if (ret != TSIP_SUCCESS) {
                WOLFSSL_MSG("R_TSIP_GenerateTlsRsaPublicKeyIndex failed");
            } else {
                /* close once */
                tsip_Close( );
                /* open again with s_inst[] */
                XMEMCPY(s_inst1, 
                    g_user_key_info.user_rsa2048_tls_pubindex.value,
                    sizeof(s_inst1));
                ret = R_TSIP_Open((uint32_t*)s_flash, s_inst1, s_inst2);
                if (ret != TSIP_SUCCESS) {
                    WOLFSSL_MSG("R_TSIP_(Re)Open failed");
                }
                 /* init vars */
                g_CAscm_Idx = (uint32_t)-1;
            }
        }
#else
        ret = R_TSIP_Open((uint32_t*)s_flash, s_inst1, s_inst2);
        if( ret != TSIP_SUCCESS ) {
            WOLFSSL_MSG("RENESAS TSIP Open failed");
        }
#endif
        /* unlock hw */
        tsip_hw_unlock();
    } else 
        WOLFSSL_MSG("Failed to lock tsip hw \n");
    
    WOLFSSL_MSG( "<< tsip_Open");
    return ret;
}

/* close TSIP driver */
void tsip_Close()
{
    WOLFSSL_MSG(">> tsip_Close");
    int ret;
    
    if ((ret = tsip_hw_lock()) == 0) {
        /* close TSIP */
        ret = R_TSIP_Close();
#if defined(WOLFSSL_RENESAS_TSIP_TLS)
        g_CAscm_Idx = (uint32_t)-1;
#endif
        /* unlock hw */
        tsip_hw_unlock();
        if( ret != TSIP_SUCCESS ) {
            WOLFSSL_MSG("RENESAS TSIP Close failed");
        }
    } else
        WOLFSSL_MSG("Failed to unlock tsip hw \n");
    WOLFSSL_MSG("<< tsip_Close");    
}

/* Support functions for TSIP TLS Capability */
#if defined(WOLFSSL_RENESAS_TSIP_TLS)

/* to inform ca certificate sign */
/* signature format expects RSA 2048 PSS with SHA256 */
void tsip_inform_cert_sign(const byte *sign)
{
    if(sign)
        ca_cert_sig = sign;
}
#if (WOLFSSL_RENESAS_TSIP_VER>=109)
void tsip_inform_user_keys_ex(
    byte*     encrypted_provisioning_key,
    byte*     iv,
    byte*     encrypted_user_tls_key,
    word32    encrypted_user_tls_key_type)
{
    WOLFSSL_MSG(">> tsip_inform_user_keys_ex");
    g_user_key_info.encrypted_provisioning_key = NULL;
    g_user_key_info.iv = NULL;
    g_user_key_info.encrypted_user_tls_key = NULL;
    
    if ( encrypted_provisioning_key ) {
        g_user_key_info.encrypted_provisioning_key = encrypted_provisioning_key;
    }
    if ( iv ) {
        g_user_key_info.iv = iv;
    }
    if ( encrypted_user_tls_key ) {
        g_user_key_info.encrypted_user_tls_key = encrypted_user_tls_key;
    }
    
    g_user_key_info.encrypted_user_tls_key_type = encrypted_user_tls_key_type;
    WOLFSSL_MSG("<< tsip_inform_user_keys_ex");
}
#elif (WOLFSSL_RENESAS_TSIP_VER>=106)
/* inform user key                                                     */
/* the function expects to be called from user application             */
/* user has to create these key information by Renesas tool in advance.*/
void tsip_inform_user_keys(
    byte *encrypted_session_key,
    byte *iv,
    byte *encrypted_user_tls_key)
{
    g_user_key_info.encrypted_session_key = NULL;
    g_user_key_info.iv = NULL;
    g_user_key_info.encrypted_user_tls_key = NULL;
    
    if ( encrypted_session_key ) {
        g_user_key_info.encrypted_session_key = encrypted_session_key;
    }
    if ( iv ) {
        g_user_key_info.iv = iv;
    }
    if ( encrypted_user_tls_key ) {
        g_user_key_info.encrypted_user_tls_key = encrypted_user_tls_key;
    }
}
#endif

#ifndef NO_WOLFSSL_RENESAS_TSIP_TLS_SESSION


/* Sha1Hmac */
int tsip_Sha1Hmac(
        const struct WOLFSSL *ssl,
        const byte* myInner, 
        word32      innerSz,
        const byte* in,
        word32      sz, 
        byte*       digest, 
        word32      verify)
{
    WOLFSSL_MSG(">> tsip_Sha1Hmac()");

    tsip_hmac_sha_handle_t _handle;
    tsip_hmac_sha_key_index_t key_index;
    int ret;

    if ((ssl == NULL) || (myInner == NULL) || (in == NULL) ||
        (digest == NULL))
      return BAD_FUNC_ARG;
    
    if ((ret = tsip_hw_lock()) != 0) {
        WOLFSSL_MSG("hw lock failed\n");
        return ret;
    }
    
    if ( (ssl->options.side == WOLFSSL_CLIENT_END && !verify) ||
         (ssl->options.side == WOLFSSL_SERVER_END &&  verify) )

        key_index = ssl->keys.tsip_client_write_MAC_secret;
    else
        key_index = ssl->keys.tsip_server_write_MAC_secret;
       
    ret = R_TSIP_Sha1HmacGenerateInit(
                    &_handle,
                    &key_index);
    
    if (ret == TSIP_SUCCESS)
        ret = R_TSIP_Sha1HmacGenerateUpdate(
                    &_handle,
                    (uint8_t*)myInner, 
                    (uint32_t)innerSz);
    
    if (ret == TSIP_SUCCESS)
        ret = R_TSIP_Sha1HmacGenerateUpdate(
                    &_handle,
                    (uint8_t*)in,
                    sz);
    
    if (ret == TSIP_SUCCESS)
        ret = R_TSIP_Sha1HmacGenerateFinal(
                    &_handle,
                    digest);
    
   
    tsip_hw_unlock();
    WOLFSSL_MSG("<< tsip_Sha1Hmac");
    return ret;
}

/* Sha256Hmac */
int tsip_Sha256Hmac(
        const struct WOLFSSL *ssl,
        const byte* myInner, 
        word32      innerSz,
        const byte* in,
        word32      sz,
        byte*       digest, 
        word32      verify)
{
    WOLFSSL_MSG(">> tsip_Sha256Hmac");

    tsip_hmac_sha_handle_t _handle;
    tsip_hmac_sha_key_index_t key_index;
    int ret;
    
    if ((ssl == NULL) || (myInner == NULL) || (in == NULL) ||
        (digest == NULL))
      return BAD_FUNC_ARG;
    
    if ( (ssl->options.side == WOLFSSL_CLIENT_END && !verify) ||
            (ssl->options.side == WOLFSSL_SERVER_END &&  verify) )

        key_index = ssl->keys.tsip_client_write_MAC_secret;
    else
        key_index = ssl->keys.tsip_server_write_MAC_secret;

    if ((ret = tsip_hw_lock()) != 0) {
        WOLFSSL_MSG("hw lock failed\n");
        return ret;
    }
    
    ret = R_TSIP_Sha256HmacGenerateInit(
                &_handle,
                &key_index);
    
    if (ret == TSIP_SUCCESS)
        ret = R_TSIP_Sha256HmacGenerateUpdate(
                &_handle,
                (uint8_t*)myInner, 
                innerSz);
    
    if (ret == TSIP_SUCCESS)
        ret = R_TSIP_Sha256HmacGenerateUpdate(
                &_handle,
                (uint8_t*)in,
                sz);
    
    if (ret == TSIP_SUCCESS)
        ret = R_TSIP_Sha256HmacGenerateFinal(
                &_handle,
                digest);
    
    /* unlock hw */
    tsip_hw_unlock();
    WOLFSSL_MSG("<< tsip_Sha256Hmac");
    return ret;
}

/* generate Verify Data based on master secret */
int tsip_generateVerifyData(const byte *ms, /* master secret */
                            const byte *side, const byte *handshake_hash,
                            byte *hashes /* out */)
{
    WOLFSSL_MSG(">> tsip_generateVerifyData");
    int ret ;
    uint32_t l_side = R_TSIP_TLS_GENERATE_CLIENT_VERIFY;
    
    if ((ms == NULL) || (side == NULL) || (handshake_hash == NULL) ||
        (hashes == NULL))
      return BAD_FUNC_ARG;
    
    if (XSTRNCMP((const char*)side, (const char*)tls_server, FINISHED_LABEL_SZ)
                                                                           == 0)
    {
        l_side = R_TSIP_TLS_GENERATE_SERVER_VERIFY;
    }
    
    if ((ret = tsip_hw_lock()) == 0) {
        ret = R_TSIP_TlsGenerateVerifyData(l_side, (uint32_t*)ms,
                       (uint8_t*)handshake_hash, hashes/* out */);
        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG("R_TSIP_TlsGenerateSessionKey failed\n");
        }
    }
    /* unlock hw */
    tsip_hw_unlock();
    WOLFSSL_MSG("<< tsip_generateVerifyData");
    return ret;
}

/* generate keys for TLS communication */
int tsip_generateSeesionKey(struct WOLFSSL *ssl)
{
    WOLFSSL_MSG(">> tsip_generateSeesionKey()");
    int ret;
    Ciphers *enc;
    Ciphers *dec;
    tsip_hmac_sha_key_index_t key_client_mac;
    tsip_hmac_sha_key_index_t key_server_mac;
    tsip_aes_key_index_t key_client_aes;
    tsip_aes_key_index_t key_server_aes;
    
    if (ssl== NULL)
      return BAD_FUNC_ARG;
      
    if ((ret = tsip_hw_lock()) == 0) {

#if (WOLFSSL_RENESAS_TSIP_VER>=109)

        uint8_t nonce[TSIP_SESSIONKEY_NONCE_SIZE] = {0};
        wc_RNG_GenerateBlock(ssl->rng, nonce,TSIP_SESSIONKEY_NONCE_SIZE);

        ret = R_TSIP_TlsGenerateSessionKey(
                    GetTsipCipherSuite(
                        ssl->options.cipherSuite0,
                        ssl->options.cipherSuite),
                    (uint32_t*)ssl->arrays->tsip_masterSecret, 
                    (uint8_t*) ssl->arrays->clientRandom,
                    (uint8_t*) ssl->arrays->serverRandom,
                    nonce,
                    &key_client_mac,
                    &key_server_mac,
                    &key_client_aes,
                    &key_server_aes,
                    NULL, NULL);        

#elif (WOLFSSL_RENESAS_TSIP_VER>=106)        

        ret = R_TSIP_TlsGenerateSessionKey(
                    _tls2tsipdef(ssl->options.cipherSuite),
                    (uint32_t*)ssl->arrays->tsip_masterSecret, 
                    (uint8_t*)ssl->arrays->clientRandom,
                    (uint8_t*)ssl->arrays->serverRandom,
                    &key_client_mac,
                    &key_server_mac,
                    &key_client_aes,
                    &key_server_aes,
                    NULL, NULL);
#endif                
        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG("R_TSIP_TlsGenerateSessionKey failed\n");
        } else {
            /* succeeded creating session keys */
            /* alloc aes instance for both enc and dec */
            enc = &ssl->encrypt;
            dec = &ssl->decrypt;
            
            if (enc) {
                if (enc->aes == NULL) {
                    enc->aes = (Aes*)XMALLOC(sizeof(Aes), ssl->heap, 
                                                    DYNAMIC_TYPE_CIPHER);
                    if (enc->aes == NULL)
                        return MEMORY_E;
                }
                
                XMEMSET(enc->aes, 0, sizeof(Aes));
            }
            if (dec) {
                if (dec->aes == NULL) {
                    dec->aes = (Aes*)XMALLOC(sizeof(Aes), ssl->heap, 
                                                    DYNAMIC_TYPE_CIPHER);
                    if (dec->aes == NULL) {
                        if (enc) {
                            XFREE(enc->aes, NULL, DYNAMIC_TYPE_CIPHER);
                        }
                        return MEMORY_E;
                    }
                }
                
                XMEMSET(dec->aes, 0, sizeof(Aes));
            }
            /* copy key index into aes */
            if (ssl->options.side == PROVISION_CLIENT) {
                XMEMCPY(&enc->aes->ctx.tsip_keyIdx, &key_client_aes, 
                                                    sizeof(key_client_aes));
                XMEMCPY(&dec->aes->ctx.tsip_keyIdx, &key_server_aes, 
                                                    sizeof(key_server_aes));
            } else {
                XMEMCPY(&enc->aes->ctx.tsip_keyIdx, &key_server_aes, 
                                                    sizeof(key_server_aes));
                XMEMCPY(&dec->aes->ctx.tsip_keyIdx, &key_client_aes, 
                                                    sizeof(key_client_aes));
            }
            /* copy hac key index into keys */
            

            ssl->keys.tsip_client_write_MAC_secret = key_client_mac;
            ssl->keys.tsip_server_write_MAC_secret = key_server_mac;
    
            /* set up key size and marked readly */
            if (enc){
                enc->aes->ctx.keySize = ssl->specs.key_size;
                /* ready for use */
                enc->setup = 1;
            }
            /* set up key size and marked readly */
            if (dec) {
                dec->aes->ctx.keySize = ssl->specs.key_size;
                /* ready for use */
                dec->setup = 1;
            }
        }
        /* unlock hw */
        tsip_hw_unlock();
    } else 
        WOLFSSL_MSG("hw lock failed\n");
    
    WOLFSSL_MSG("<< tsip_generateSeesionKey");
    return ret;
}
/* generate Master secrete by TSIP */
#if (WOLFSSL_RENESAS_TSIP_VER>=109)

int tsip_generateMasterSecretEx(
        byte        cipherSuiteFirst,
        byte        cipherSuite,
        const byte *pr, /* pre-master    */
        const byte *cr, /* client random */
        const byte *sr, /* server random */
        byte *ms)
{
    WOLFSSL_MSG(">> tsip_generateMasterSecretEx");
    int ret;
    
    if ((pr == NULL) || (cr == NULL) || (sr == NULL) ||
        (ms == NULL))
      return BAD_FUNC_ARG;
      
    uint32_t tsipCS = GetTsipCipherSuite(cipherSuiteFirst,cipherSuite );
    if( tsipCS == 0xffffffff)
        return BAD_FUNC_ARG;

    if ((ret = tsip_hw_lock()) == 0) {
        ret = R_TSIP_TlsGenerateMasterSecret( 
            tsipCS,
            (uint32_t*)pr,
            (uint8_t*)cr, (uint8_t*)sr, (uint32_t*)ms);
        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG("R_TSIP_TlsGenerateMasterSecret failed\n");
        }
        /* unlock hw */
        tsip_hw_unlock();
    } else {
        WOLFSSL_MSG(" hw lock failed ");
    }
    WOLFSSL_MSG("<< tsip_generateMasterSecretEx");
    return ret;
}

#elif (WOLFSSL_RENESAS_TSIP_VER>=106)

int tsip_generateMasterSecret(
        const byte* pr, /* pre-master    */
        const byte* cr, /* client random */
        const byte* sr, /* server random */
        byte*       ms)
{
    int ret;
    
    if ((pr == NULL) || (cr == NULL) || (sr == NULL) ||
        (ms == NULL))
      return BAD_FUNC_ARG;
      
    if ((ret = tsip_hw_lock()) == 0) {
        ret = R_TSIP_TlsGenerateMasterSecret( 
                (uint32_t*)pr,
                (uint8_t*)cr,
                (uint8_t*)sr,
                (uint32_t*)ms);

        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG("R_TSIP_TlsGenerateMasterSecret failed\n");
        }
        /* unlock hw */
        tsip_hw_unlock();
    } else {
        WOLFSSL_MSG(" hw lock failed ");
    }
    
    return ret;
}
#endif
/* generate pre-Master secrete by TSIP */
int tsip_generatePremasterSecret(byte *premaster, word32 preSz )
{
    WOLFSSL_MSG(">> tsip_generatePremasterSecret");
    int ret;
    
    if (premaster == NULL)
      return BAD_FUNC_ARG;
    
    if ((ret = tsip_hw_lock()) == 0 && preSz >=
                                    (R_TSIP_TLS_MASTER_SECRET_WORD_SIZE*4)) {
        /* generate pre-master, 80 bytes */
        ret = R_TSIP_TlsGeneratePreMasterSecret( (uint32_t*)premaster );
        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG(" R_TSIP_TlsGeneratePreMasterSecret failed\n");
        }
        
        /* unlock hw */
        tsip_hw_unlock();
    } else {
        WOLFSSL_MSG(" hw lock failed or preSz is smaller than 80");
    }
    WOLFSSL_MSG("<< tsip_generatePremasterSecret");
    return ret;
}

/* 
* generate encrypted pre-Master secrete by TSIP
*/
int tsip_generateEncryptPreMasterSecret(
        WOLFSSL*    ssl,
        byte*       out,
        word32*     outSz)
{
    WOLFSSL_MSG(">> tsip_generateEncryptPreMasterSecret");
    int ret;
    
    if ((ssl == NULL) || (out == NULL) || (outSz == NULL))
      return BAD_FUNC_ARG;
    
    if ((ret = tsip_hw_lock()) == 0) {
        if (*outSz >= 256)
           
            #if  (WOLFSSL_RENESAS_TSIP_VER>=109)
           
            ret = R_TSIP_TlsEncryptPreMasterSecretWithRsa2048PublicKey(
                        (uint32_t*)ssl->peerTsipEncRsaKeyIndex,
                        (uint32_t*)&ssl->arrays->preMasterSecret[VERSION_SZ],
                        (uint8_t*)out);

            #elif (WOLFSSL_RENESAS_TSIP_VER>=106)
            
            ret = R_TSIP_TlsEncryptPreMasterSecret(
                          (uint32_t*)ssl->peerTsipEncRsaKeyIndex,
                          (uint32_t*)&ssl->arrays->preMasterSecret[VERSION_SZ],
                          (uint8_t*)out);
            
            #endif
        else
            ret = -1;
            
        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG(" R_TSIP_TlsEncryptPreMasterSecret failed\n");
        } else {
            *outSz = 256; /* TSIP can only handles 2048 RSA */
        }
        
        tsip_hw_unlock();

    } else {
        WOLFSSL_MSG(" hw lock failed ");
    }
    WOLFSSL_MSG("<< tsip_generateEncryptPreMasterSecret");
    return ret;
}
#endif /* NO_WOLFSSL_RENESAS_TSIP_TLS_SESSION */

/* Certificate verification by TSIP */
int tsip_tls_CertVerify(
        const byte* cert,       word32 certSz,
        const byte* signature,  word32 sigSz,
        word32      key_n_start,word32 key_n_len,
        word32      key_e_start,word32 key_e_len,
        byte*       tsip_encRsaKeyIndex)
{
    WOLFSSL_MSG(">> tsip_tls_CertVerify");
    int ret;
    
    if (cert == NULL)
      return BAD_FUNC_ARG;
    
    if (!signature) {
        WOLFSSL_MSG(" signature for ca verification is not set\n");
        return -1;
    }
    if (!tsip_encRsaKeyIndex) {
        WOLFSSL_MSG(" tsip_encRsaKeyIndex is NULL.\n");
        return -1;
    }
    
    if ((ret = tsip_hw_lock()) == 0) {

        #if (WOLFSSL_RENESAS_TSIP_VER>=109)

         ret = R_TSIP_TlsCertificateVerification(
                g_user_key_info.encrypted_user_tls_key_type,
                (uint32_t*)g_encrypted_publicCA_key,/* encrypted public key  */
                (uint8_t*)cert,                    /* certificate der        */
                certSz,                            /* length of der          */
                (uint8_t*)signature,               /* sign data by RSA PSS   */
                key_n_start,  /* start position of public key n in bytes     */
                (key_n_start + key_n_len),     /* length of the public key n */
                key_e_start,                   /* start pos, key e in bytes  */
                (key_e_start + key_e_len),     /* length of the public key e */
                (uint32_t*)tsip_encRsaKeyIndex /* returned encrypted key     */
                );

        #elif (WOLFSSL_RENESAS_TSIP_VER>=106)

        ret = R_TSIP_TlsCertificateVerification(
                (uint32_t*)g_encrypted_publicCA_key,/* encrypted public key  */
                (uint8_t*)cert,                    /* certificate der        */
                certSz,                            /* length of der          */
                (uint8_t*)signature,               /* sign data by RSA PSS   */
                key_n_start,  /* start position of public key n in bytes     */
                (key_n_start + key_n_len),     /* length of the public key n */
                key_e_start,                   /* start pos, key e in bytes  */
                (key_e_start + key_e_len),     /* length of the public key e */
                (uint32_t*)tsip_encRsaKeyIndex /* returned encrypted key     */
                );
        #endif

        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG(" R_TSIP_TlsCertificateVerification() failed");
        }
        tsip_hw_unlock();
    } else {
        WOLFSSL_MSG(" hw lock failed ");
    }
    WOLFSSL_MSG("<< tsip_tls_CertVerify");
    return ret;
}
/* Root Certificate verification */
int tsip_tls_RootCertVerify(
        const byte* cert,           word32 cert_len,
        word32      key_n_start,    word32 key_n_len,
        word32      key_e_start,    word32 key_e_len,
        word32      cm_row)
{
    WOLFSSL_MSG(">> tsip_tls_RootCertVerify");
    int ret;
    /* call to generate encrypted public key for certificate verification */
    uint8_t *signature = (uint8_t*)ca_cert_sig;
    
    if (cert == NULL)
      return BAD_FUNC_ARG;
      
    if (!signature) {
        WOLFSSL_MSG(" signature for ca verification is not set\n");
        return -1;
    }
    
    if ((ret = tsip_hw_lock()) == 0) {

        #if (WOLFSSL_RENESAS_TSIP_VER>=109)

        ret = R_TSIP_TlsRootCertificateVerification(
                g_user_key_info.encrypted_user_tls_key_type,            
                (uint8_t*)cert,             /* CA cert */            
                (uint32_t)cert_len,         /* length of CA cert */            
                key_n_start,                /* Byte position of public key */
                (key_n_start + key_n_len),
                key_e_start,
                (key_e_start + key_e_len),
                (uint8_t*)ca_cert_sig,      /* "RSA 2048 PSS with SHA256" */
                g_encrypted_publicCA_key);  /* RSA-2048 public key 560 bytes */

        #elif (WOLFSSL_RENESAS_TSIP_VER>=106)

        ret = R_TSIP_TlsRootCertificateVerification(                         
                (uint8_t*)cert,/* CA cert */
                (uint32_t)cert_len,/* length of CA cert */
                key_n_start, /* Byte position of public key */
                (key_n_start + key_n_len),
                key_e_start,
                (key_e_start + key_e_len),
                (uint8_t*)ca_cert_sig,/* "RSA 2048 PSS with SHA256" */
                /* RSA-2048 public key used by
                    RSA-2048 PSS with SHA256. 560 Bytes*/
                g_encrypted_publicCA_key );
        
        #endif
        
        if (ret != TSIP_SUCCESS) {
            WOLFSSL_MSG(" R_TSIP_TlsRootCertVerify() failed");
        } else {
            g_CAscm_Idx = cm_row;
        }
        
        tsip_hw_unlock();
    } else {
        WOLFSSL_MSG(" hw lock failed ");
    }
    WOLFSSL_MSG("<< tsip_tls_RootCertVerify");
    return ret;
}
#endif /* WOLFSSL_RENESAS_TSIP_TLS */

#ifdef WOLFSSL_RENESAS_TSIP_CRYPT_DEBUG

/* err
 * e_tsip_err
    TSIP_SUCCESS = 0, 
    TSIP_ERR_SELF_CHECK1,  // Self-check 1 fail or TSIP function internal err.
    TSIP_ERR_RESOURCE_CONFLICT, // A resource conflict occurred.
    TSIP_ERR_SELF_CHECK2,       // Self-check 2 fail.
    TSIP_ERR_KEY_SET,           // setting the invalid key.
    TSIP_ERR_AUTHENTICATION,    // Authentication failed.
    TSIP_ERR_CALLBACK_UNREGIST, // Callback function is not registered.
    TSIP_ERR_PARAMETER,         // Illegal Input data.
    TSIP_ERR_PROHIBIT_FUNCTION, // An invalid function call occurred.
 *  TSIP_RESUME_FIRMWARE_GENERATE_MAC,  
                  // There is a continuation of R_TSIP_GenerateFirmwareMAC.
*/

static void hexdump(const uint8_t* in, uint32_t len)
{
    uint32_t i;

    if (in == NULL)
        return;

    for (i = 0; i <= len;i++, in++){
        printf("%02x:", *in);
        if (((i+1)%16)==0){
            printf("\n");
        }
    }
    printf("\n");
}

byte *ret2err(word32 ret)
{
    switch(ret){
        case TSIP_SUCCESS:     return "success";
        case TSIP_ERR_SELF_CHECK1: return "selfcheck1";
        case TSIP_ERR_RESOURCE_CONFLICT: return "rsconflict";
        case TSIP_ERR_SELF_CHECK2: return "selfcheck2";
        case TSIP_ERR_KEY_SET: return "keyset";
        case TSIP_ERR_AUTHENTICATION: return "authentication";
        case TSIP_ERR_CALLBACK_UNREGIST: return "callback unreg";
        case TSIP_ERR_PARAMETER: return "badarg";
        case TSIP_ERR_PROHIBIT_FUNCTION: return "prohibitfunc";
        case TSIP_RESUME_FIRMWARE_GENERATE_MAC: return "conti-generate-mac";
        default:return "unknown";
    }
}

#endif /* WOLFSSL_RENESAS_TSIP_CRYPT_DEBUG */
#endif /* WOLFSSL_RENESAS_TSIP */
