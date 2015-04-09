/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __CONFIG_H__
#define __CONFIG_H__

/* Define if MPI driver must uses kernel module spi-bcm63xx (which is much slower) */
#undef BCMPH_USE_SPI_DRIVER
/* Define to enable the usage of interrupts in PCM code */
#undef BCMPH_ENABLE_PCM_INTERRUPTS
/* Define to trace MPI transfers */
#undef BCMPH_DEBUG_MPI
/* Define to use option VP_OPTION_ID_PULSE in MicroSemi VoicePath API */
#undef BCMPH_VP_DECODE_PULSE

#ifndef __KERNEL__
# ifndef BCMPH_NOHW
#  define BCMPH_NOHW
# endif /* BMCPH_NOHW */
#endif /* !__KERNEL__ */

#ifdef BCMPH_NOHW
# ifndef BCMPH_TEST_PCM
#  define BCMPH_TEST_PCM
# endif /* !BCMPH_TEST_PCM */
# ifdef BCMPH_ENABLE_PCM_INTERRUPTS
#  undef BCMPH_ENABLE_PCM_INTERRUPTS
# endif /* BCMPH_ENABLE_PCM_INTERRUPTS */
#endif /* BCMPH_NOHW */

#ifdef BCMPH_TEST_PCM
# ifndef BCMPH_DEBUG
#  define BCMPH_DEBUG
# endif /* !BCMPH_DEBUG */
# undef BCMPH_DEBUG_MPI
#endif /* !BCMPH_TEST_PCM */

#ifndef BCMPH_DEBUG
#undef BCMPH_DEBUG_MPI
#endif

#ifndef __KERNEL__
#undef BCMPH_DEBUG_MPI
#endif

#endif /* __CONFIG_H__ */
