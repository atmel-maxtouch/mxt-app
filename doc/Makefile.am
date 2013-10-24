EXTRA_DIST = doxygen.cfg.in
DISTCLEANFILES = doxygen.cfg

all: doxygen

doxygen.cfg: $(srcdir)/doxygen.cfg.in
	-rm -f $@
	sed 's|@TOP_SRCDIR@|$(top_srcdir)|g' $< > $@

.PHONY: doxygen
doxygen: doxygen.cfg
	doxygen doxygen.cfg 1>doxygen.log

clean: clean-local
clean-local:
	-rm -Rf html
