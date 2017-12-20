/*
 * wolfssl_example.h
 *
 *  Created on: Oct 3, 2016
 *      Author: davidgarske
 */

#ifndef WOLFSSL_EXAMPLE_H_
#define WOLFSSL_EXAMPLE_H_

#include <stm32f4xx_hal.h>
#include <stm32f4xx.h>
#include <cmsis_os.h>

#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#ifndef WOLFSSL_USER_SETTINGS
	#include <wolfssl/options.h>
#endif
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/ssl.h>
#include <wolfcrypt/test/test.h>
#include <wolfcrypt/benchmark/benchmark.h>
#include <wolfssl/wolfcrypt/tpm.h>


void wolfCryptDemo(void const * argument);


#endif /* WOLFSSL_EXAMPLE_H_ */
