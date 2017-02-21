AUTOMAKE_OPTIONS = foreign

#CFLAGS = -g -DDEBUG -Wall -pedantic -std=c99
CFLAGS = -Wall -pedantic -std=c99
EXTRA_DIST = pi-blaster.service

sbin_PROGRAMS = pi-blaster
pi_blaster_SOURCES = pi-blaster.c mailbox.c

if !DEBIAN_PKG
if HAVE_SYSTEMD
systemdsystemunit_DATA = pi-blaster.service

# Unless we are cross-compiling, start now.
if !CROSS_COMPILING
install-data-hook:
	systemctl enable pi-blaster
	systemctl start pi-blaster

uninstall-hook:
	systemctl stop pi-blaster
	systemctl disable pi-blaster
endif # !CROSS_COMPILING

else
init_ddir = $(sysconfdir)/init.d
init_d_SCRIPTS = pi-blaster.boot.sh

# Unless we are cross-compiling, start now.
if !CROSS_COMPILING
install-data-hook:
	update-rc.d pi-blaster.boot.sh defaults
	$(init_ddir)/pi-blaster.boot.sh start

uninstall-hook:
	update-rc.d pi-blaster.boot.sh remove
	killall pi-blaster
endif # !CROSS_COMPILING

endif # !HAVE_SYSTEMD
endif # !DEBIAN_PKG
