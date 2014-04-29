dnl Process this file with autoconf to produce a configure script
AC_INIT([osmo-conv-test],
	[0.1],
	[tom@tsou.cc])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([dist-bzip2])
LT_INIT

dnl kernel style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl checks for programs
AM_PROG_AS
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_INSTALL

dnl checks for header files
AC_HEADER_STDC

dnl checks for processor family 
AX_EXT

dnl checks for libraries
PKG_CHECK_MODULES(LIBOSMOCORE, libosmocore  >= 0.3.9)

AC_OUTPUT(
    src/Makefile
    tests/Makefile
    Makefile)
