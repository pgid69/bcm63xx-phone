/***************************************************************************
*
*    Filename: countryArchive.h
*
****************************************************************************
*    Description:
*
*     This file provides a central list or archive of supported countries.
*     It is intended to be used to generate an enumerated type of countries.
*     It MUST be included by another wrapper header file which provides
*     the enum skeleton and macro to generate enumerated type names. The
*     enum macro that MUST be defined in the containing header file is
*     'COUNTRY_ARCHIVE_MAKE_NAME'. For example, the containing header file
*     should declare something similar to the following:
*
*        #define COUNTRY_ARCHIVE_MAKE_NAME( country )    MY_COUNTRY_##country
*
*        typedef enum
*        {
*           #include <countryArchive.h>
*
*        } MY_COUNTRY;
*
*
*     This example would generate a list of countries as follows:
*
*        enum
*        {
*           MY_COUNTRY_JAPAN,
*           MY_COUNTRY_GERMANY,
*           MY_COUNTRY_SWEDEN,
*           ...
*        } MY_COUNTRY;
*
*     Using this method allows the list of countries to be maintained in a
*     central location. However, modules are free to create their own country
*     name-space by providing a wrapper for this file.
*
****************************************************************************/

/*
** Include guards have been purposefully excluded. This header MUST be included
** by a wrapper header file which provides the inclusion protection.
*/

#ifndef COUNTRY_ARCHIVE_MAKE_NAME
   #error "COUNTRY_ARCHIVE_MAKE_NAME must be defined!"
#endif

COUNTRY_ARCHIVE_MAKE_NAME( ETSI ) /* ETSI */
COUNTRY_ARCHIVE_MAKE_NAME( GR57 ) /* Telcordia GR-57 */
COUNTRY_ARCHIVE_MAKE_NAME( AT   ) /* Austria */
COUNTRY_ARCHIVE_MAKE_NAME( AU   ) /* Australia */
COUNTRY_ARCHIVE_MAKE_NAME( BE   ) /* Belgium */
COUNTRY_ARCHIVE_MAKE_NAME( BG   ) /* Bulgaria */
COUNTRY_ARCHIVE_MAKE_NAME( BR   ) /* Brazil */
COUNTRY_ARCHIVE_MAKE_NAME( CA   ) /* Canada */
COUNTRY_ARCHIVE_MAKE_NAME( CH   ) /* Switzerland */
COUNTRY_ARCHIVE_MAKE_NAME( CN   ) /* China */
COUNTRY_ARCHIVE_MAKE_NAME( DE   ) /* Germany */
COUNTRY_ARCHIVE_MAKE_NAME( DK   ) /* Danemark */
COUNTRY_ARCHIVE_MAKE_NAME( ES   ) /* Spain */
COUNTRY_ARCHIVE_MAKE_NAME( FI   ) /* Finland */
COUNTRY_ARCHIVE_MAKE_NAME( FR   ) /* France */
COUNTRY_ARCHIVE_MAKE_NAME( GB   ) /* United Kingdom */
COUNTRY_ARCHIVE_MAKE_NAME( GR   ) /* Greece */
COUNTRY_ARCHIVE_MAKE_NAME( HK   ) /* Hong Kong */
COUNTRY_ARCHIVE_MAKE_NAME( HU   ) /* Hungary */
COUNTRY_ARCHIVE_MAKE_NAME( IE   ) /* Ireland */
COUNTRY_ARCHIVE_MAKE_NAME( IL   ) /* Israel */
COUNTRY_ARCHIVE_MAKE_NAME( IS   ) /* Iceland */
COUNTRY_ARCHIVE_MAKE_NAME( IT   ) /* Italy */
COUNTRY_ARCHIVE_MAKE_NAME( JP   ) /* Japan */
COUNTRY_ARCHIVE_MAKE_NAME( KR   ) /* Korea */
COUNTRY_ARCHIVE_MAKE_NAME( NL   ) /* Netherlands */
COUNTRY_ARCHIVE_MAKE_NAME( NO   ) /* Norway */
COUNTRY_ARCHIVE_MAKE_NAME( NZ   ) /* New Zealand */
COUNTRY_ARCHIVE_MAKE_NAME( PT   ) /* Portugal */
COUNTRY_ARCHIVE_MAKE_NAME( RU   ) /* Russia */
COUNTRY_ARCHIVE_MAKE_NAME( SE   ) /* Sweden */
COUNTRY_ARCHIVE_MAKE_NAME( SG   ) /* Singapore */
COUNTRY_ARCHIVE_MAKE_NAME( TK   ) /* Turkey */
COUNTRY_ARCHIVE_MAKE_NAME( TW   ) /* Taiwan */
COUNTRY_ARCHIVE_MAKE_NAME( US   ) /* United States */
COUNTRY_ARCHIVE_MAKE_NAME( ZA   ) /* South Africa */

#undef COUNTRY_ARCHIVE_MAKE_NAME

