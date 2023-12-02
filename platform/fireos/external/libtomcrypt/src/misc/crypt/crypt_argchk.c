/* LibTomCrypt, modular cryptographic library -- Tom St Denis
 *
 * LibTomCrypt is a library that provides various cryptographic
 * algorithms in a highly modular and flexible manner.
 *
 * The library is free for all purposes without any express
 * guarantee it works.
 *
 * Tom St Denis, tomstdenis@gmail.com, http://libtom.org
 */

/*
 * Portions of this file are copyright (c) 2017 Amazon.com, Inc. or its affiliates.  All rights reserved.
 */

#include "tomcrypt.h"

/**
  @file crypt_argchk.c
  Perform argument checking, Tom St Denis
*/

#if (ARGTYPE == 0)
void crypt_argchk(char *v, char *s, int d)
{
#if defined(CONFIG_UFBL) && !defined(CONFIG_UFBL_FUZZER)
 /*
  * This is the block the bootloader will use on a device.
  * Failing this check will cause the device to hang, preventing
  * an attacker from doing anything else with the device.
  */
 printf("LTC_ARGCHK '%s' failure on line %d of file %s\n",
        v, d, s);
 while(1);
#else
 fprintf(stderr, "LTC_ARGCHK '%s' failure on line %d of file %s\n",
        v, d, s);
#if defined(CONFIG_UFBL_FUZZER)
 /*
  * This is a fuzzer scenario
  * AFL recognizes abort() as a crash,
  * exit is used to avoid false positives
  */
 exit(-1);
#else
 /* This is a normal scenario */
 abort();
#endif
#endif
}
#endif

/* $Source$ */
/* $Revision$ */
/* $Date$ */
