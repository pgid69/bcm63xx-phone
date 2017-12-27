#ifndef __MACROS_H__
#define __MACROS_H__

#if defined(__GNUC__) && defined(__GNUC_MINOR__)
#define GNUC_VERSION \
   (__GNUC__ << 16) + __GNUC_MINOR__
#define GNUC_PREREQ(maj, min) \
   (GNUC_VERSION >= ((maj) << 16) + (min))
#else
#define GNUC_PREREQ(maj, min) 0
#endif

#ifndef BUILD_BUG_ON_ZERO
# define BUILD_BUG_ON_ZERO(e) \
  (sizeof(struct { int:-!!(e)*1234; }))
#endif

#if GNUC_PREREQ(3, 1)
#define SAME_TYPE(a, b) \
   __builtin_types_compatible_p(typeof(a), typeof(b))
#define MUST_BE_ARRAY(a) \
   BUILD_BUG_ON_ZERO(SAME_TYPE((a), &(*a)))
#else
#define MUST_BE_ARRAY(a) \
   BUILD_BUG_ON_ZERO(sizeof(a) % sizeof(*a))
#endif

#ifndef ARRAY_SIZE
# ifdef __cplusplus
template <typename T, size_t N>
char ( &ARRAY_SIZE_HELPER( T (&array)[N] ))[N];
#  define ARRAY_SIZE( array ) \
    (sizeof( ARRAY_SIZE_HELPER( array ) ))
# else
#  define ARRAY_SIZE(a) ( \
   (sizeof(a) / sizeof(*a)) \
   + MUST_BE_ARRAY(a))
# endif
#endif /* !ARRAY_SIZE */

#ifndef container_of
# define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})
#endif /* container_of */

#endif /* __MACROS_H__ */
