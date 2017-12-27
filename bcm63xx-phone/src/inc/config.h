/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 Define if MPI driver must uses the kernel driver spi-bcm63xx
 Kernel driver spi-bcm63xx must be patched to add the function
 bcm63xx_spi_raw_sync_locked().
 Without this function, spi-bcm63xx is too slow mainly because it does
 not allow toggling CS signal between each byte, forcing to have one
 spi_transfer for each byte
*/
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
# ifdef BCMPH_DAHDI_DRIVER
#  error "BCMPH_DAHDI_DRIVER can't be defined if __KERNEL__ is undefined"
# endif
#endif /* !__KERNEL__ */

#ifdef BCMPH_NOHW
# ifndef BCMPH_TEST_PCM
#  define BCMPH_TEST_PCM
# endif /* !BCMPH_TEST_PCM */
# ifdef BCMPH_ENABLE_PCM_INTERRUPTS
#  error "BCMPH_ENABLE_PCM_INTERRUPTS is incompatible with BCMPH_NOHW"
# endif /* BCMPH_ENABLE_PCM_INTERRUPTS */
#endif /* BCMPH_NOHW */

#ifdef BCMPH_TEST_PCM
# ifndef BCMPH_DEBUG
#  define BCMPH_DEBUG
# endif /* !BCMPH_DEBUG */
# ifdef BCMPH_DEBUG_MPI
#  error "BCMPH_DEBUG_MPI is incompatible with BCMPH_TEST_PCM"
# endif
#endif /* !BCMPH_TEST_PCM */

#ifndef BCMPH_DEBUG
# ifdef BCMPH_DEBUG_MPI
#  error "BCMPH_DEBUG_MPI can't be defined if BCMPH_DEBUG is undefined"
# endif
#endif

#ifndef __KERNEL__
# ifdef BCMPH_DEBUG_MPI
#  error "BCMPH_DEBUG_MPI can't be defined if __KERNEL__ is undefined"
# endif
#endif

#ifdef BCMPH_DAHDI_DRIVER
# ifdef BCMPH_EXPORT_DEV_FILE
#  error "BCMPH_DAHDI is incompatible with BCMPH_EXPORT_DEV_FILE"
# endif
#endif

#if ((!defined BCMPH_DAHDI_DRIVER) && (!defined BCMPH_EXPORT_DEV_FILE))
# define BCMPH_EXPORT_DEV_FILE
#endif

#endif /* __CONFIG_H__ */
