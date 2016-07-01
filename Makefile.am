bin_PROGRAMS = mw-mavlink 
mw_mavlink_SOURCES = main.c udp.c mw.c mavlink.c params.c gamepad.c
mw_mavlink_CFLAGS = -Wall
mw_mavlink_LDFLAGS = 
mw_mavlink_LDADD = -lmw_core -lrt -lpthread -lssl -lcrypto -lresolv -lm $(libconfig_LIBS)

mwconfdir=$(sysconfdir)/mw
mwbindir=$(bindir)

install-exec-hook:
	mkdir -p $(mwconfdir)
	cp utils/mw-mavlink.start $(mwconfdir)/
	chmod 666 $(mwconfdir)/mw-mavlink.start
	cp utils/camera_streamer.sh $(mwbindir)/
	chmod 777 $(mwbindir)/camera_streamer.sh
