AC_INIT([mw-mavlink],[0.0.1],[gregory.dymarek@gmail.com])
AM_INIT_AUTOMAKE([1.9 foreign])

AC_ARG_ENABLE([config], AS_HELP_STRING([--disable-config], [Build without libconfig support]))

if test "x$enable_config" != "xno"; then
   PKG_CHECK_MODULES([libconfig], [libconfig >= 1.4.9])
   AC_DEFINE([CFG_ENABLED])
fi

AC_ARG_ENABLE([rpicam], AS_HELP_STRING([--disable-rpicam], [Build without RPi camera support]))

if test "x$enable_rpicam" != "xno"; then
   AC_DEFINE([RPICAM_ENABLED])
fi

AC_PROG_CC

AC_CONFIG_FILES([Makefile])

AC_OUTPUT

cp confdefs.h def.h
