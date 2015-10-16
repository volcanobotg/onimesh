CC = g++
CFLAGS = -o
FILES = onimesh.cpp onimeshfunctions.cpp
DEPS = onimeshfunctions.h
OUT_EXE = onimesh
SRCDIR = src
BINDIR = bin

onimesh: $(SRCDIR)/$(FILES) $(SRCDIR)/$(DEPS)
	$(CC) $(CFLAGS) $(BINDIR)/$(OUT_EXE) $(SRCDIR)/$(FILES)
	@echo onimesh compilation complete

clean:
	rm -f *.o core
	@echo Project has been cleaned

rebuild: clean all
	@echo Cleaned and recompiled