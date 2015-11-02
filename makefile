#OpenNI2 variables
OPENNI2_INCLUDE=/Applications/OpenNI-MacOSX-x64-2.2/Include
OPENNI2_REDIST=/Applications/OpenNI-MacOSX-x64-2.2/Redist
OPENNI2_DLIBRARY=/Applications/OpenNI-MacOSX-x64-2.2/Redist/libOpenNI2.dylib

#flags
CC=g++
CFLAGS=-c -Wall -I$(OPENNI2_INCLUDE)
LDFLAGS=
EXECUTABLE_NAME=onimesh

# Folders
SRC=src
BIN=bin
OBJ=$(BIN)/obj

#functions to get source files
SOURCES = $(wildcard $(SRC)/*.cpp)
EXECUTABLE_FILES = $(EXECUTABLE_NAME:%=$(BIN)/%)
OBJECT_FILES     = $(SOURCES:%.cpp=$(OBJ)/%.o)

#name of target: first target in the file so it is called first
build: $(EXECUTABLE_FILES)

#target: to clean up-has to be called in command
clean:
	rm -r -f $(BIN)

#fake target- might not be needed
.PHONY: build clean

#linking-called by build target
$(EXECUTABLE_FILES): $(OBJECT_FILES)
	#linking of all object files
	$(CC) $(OPENNI2_DLIBRARY) $(LDFLAGS) -o $@ $^
	#copy OPENNI2_REDIST to BIN
	rsync -rupE $(OPENNI2_REDIST)/ $(BIN)
	#just send a message
	@echo "Build successful!"

#compiling
$(OBJECT_FILES): $(OBJ)/%.o: %.cpp
	#just a message
	@echo Compiling $<
	#makes object directory for all compiled objects
	mkdir -p $(@D)
	#actual compile command
	$(CC) $(CFLAGS) -o $@ $<



