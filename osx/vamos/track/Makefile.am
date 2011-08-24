lib_LTLIBRARIES = libvamos-track.la

LIBS =

libvamos_track_la_SOURCES = \
	Gl_Road_Segment.cc \
	Road_Segment.cc \
	Strip_Track.cc \
	Strip_Track_Reader.cc

pkginclude_HEADERS = \
	Gl_Road_Segment.h \
	Road_Segment.h \
	Strip_Track.h \
	Track.h

pkgincludedir = $(includedir)/@PACKAGE@/track

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -Wall -pedantic -O2 -g -DBOOST_TEST_DYN_LINK -L/usr/lib/boost

if DO_UNIT_TESTS
UNIT_TESTS = test_Road_Segment
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = test_Road_Segment
test_Road_Segment_SOURCES = test_Road_Segment.cc
test_Road_Segment_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	Road_Segment.lo \
	-lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	./$(UNIT_TESTS)
endif
