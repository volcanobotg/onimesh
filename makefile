OPENNI2_INCLUDE=/Applications/OpenNI-MacOSX-x64-2.2/Include
OPENNI2_REDIST=/Applications/OpenNI-MacOSX-x64-2.2/Redist
OPENNI2_DLIBRARY=/Applications/OpenNI-MacOSX-x64-2.2/Redist/libOpenNI2.dylib

CC=g++
CFLAGS=-c -Wall -I$(OPENNI2_INCLUDE)
LDFLAGS=
EXECUTABLE_NAME=onimesh

# Folders
SRC=src
BIN=bin
OBJ=$(BIN)/obj

SOURCES = $(wildcard $(SRC)/*.cpp)
EXECUTABLE_FILES = $(EXECUTABLE_NAME:%=$(BIN)/%)
OBJECT_FILES     = $(SOURCES:%.cpp=$(OBJ)/%.o)

build: $(EXECUTABLE_FILES)

clean:
	rm -r -f $(BIN)

.PHONY: build clean

$(EXECUTABLE_FILES): $(OBJECT_FILES)
	$(CC) $(OPENNI2_DLIBRARY) $(LDFLAGS) -o $@ $^
	rsync -rupE $(OPENNI2_REDIST)/ $(BIN)
	@echo "Build successful!"

$(OBJECT_FILES): $(OBJ)/%.o: %.cpp
	@echo Compiling $<
	mkdir -p $(@D)
	$(CC) $(CFLAGS) -o $@ $<

