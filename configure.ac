AC_INIT([mw-mavlink],[0.0.1],[gregory.dymarek@gmail.com])
AM_INIT_AUTOMAKE([1.9 foreign])

AC_ARG_WITH([config], AS_HELP_STRING([--with-config], [Build with support for saving configuration (requires libconfig)]))

if test "x$with_config" = "xyes"; then
   PKG_CHECK_MODULES([libconfig], [libconfig >= 1.4.9])
   AC_DEFINE([CFG_ENABLED])
fi

AC_PROG_CC

AC_CONFIG_FILES([Makefile])

AC_OUTPUT

cp confdefs.h def.h
