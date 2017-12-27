/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
/* Enable extra warnings corresponding to option -Wextra */
#pragma GCC diagnostic warning "-Wall"

#pragma GCC diagnostic warning "-Wclobbered"
#pragma GCC diagnostic warning "-Wempty-body"
#pragma GCC diagnostic warning "-Wignored-qualifiers"
#pragma GCC diagnostic warning "-Wmissing-field-initializers"
#pragma GCC diagnostic warning "-Wmissing-parameter-type"
#pragma GCC diagnostic warning "-Wold-style-declaration"
#pragma GCC diagnostic warning "-Woverride-init"
#pragma GCC diagnostic warning "-Wsign-compare"
#ifdef BCMPH_NOHW
# pragma GCC diagnostic warning "-Wshift-negative-value"
#endif // BCMPH_NOHW
#pragma GCC diagnostic warning "-Wswitch-default"
//#pragma GCC diagnostic warning "-Wswitch-enum"
#pragma GCC diagnostic warning "-Wtype-limits"
#pragma GCC diagnostic warning "-Wuninitialized"
#pragma GCC diagnostic warning "-Wunused-but-set-parameter"
#pragma GCC diagnostic warning "-Wunused-but-set-variable"
//#pragma GCC diagnostic warning "-Wunused-parameter"
