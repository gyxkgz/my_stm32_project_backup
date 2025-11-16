#pragma once
#include "main.h"
#ifdef USE_ITCM_RAM
#if defined(ITCM_RAM_OPTIMISATION) && !defined(DEBUG)
#define FAST_CODE                   __attribute__((section(".tcm_code"))) __attribute__((optimize(ITCM_RAM_OPTIMISATION)))
#else
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#endif
// Handle case where we'd prefer code to be in ITCM, but it won't fit on the device
#ifndef FAST_CODE_PREF
#define FAST_CODE_PREF              FAST_CODE
#endif

#define FAST_CODE_NOINLINE          NOINLINE

#else
#define FAST_CODE
#define FAST_CODE_PREF
#define FAST_CODE_NOINLINE
#endif // USE_ITCM_RAM