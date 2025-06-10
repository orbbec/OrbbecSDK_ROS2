
#ifndef OB_EXPORT_H
#define OB_EXPORT_H

#ifdef OB_STATIC_DEFINE
#  define OB_EXPORT
#  define OB_NO_EXPORT
#else
#  ifndef OB_EXPORT
#    ifdef OrbbecSDK_EXPORTS
        /* We are building this library */
#      define OB_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define OB_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef OB_NO_EXPORT
#    define OB_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef OB_DEPRECATED
#  define OB_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef OB_DEPRECATED_EXPORT
#  define OB_DEPRECATED_EXPORT OB_EXPORT OB_DEPRECATED
#endif

#ifndef OB_DEPRECATED_NO_EXPORT
#  define OB_DEPRECATED_NO_EXPORT OB_NO_EXPORT OB_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef OB_NO_DEPRECATED
#    define OB_NO_DEPRECATED
#  endif
#endif

#endif /* OB_EXPORT_H */
