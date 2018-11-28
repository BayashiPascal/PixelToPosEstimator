# Build mode
# 0: development (max safety, no optimisation)
# 1: release (min safety, optimisation)
# 2: fast and furious (no safety, optimisation)
BUILD_MODE?=1

all: pbmake_wget main
	
# Automatic installation of the repository PBMake in the parent folder
pbmake_wget:
	if [ ! -d ../PBMake ]; then wget https://github.com/BayashiPascal/PBMake/archive/master.zip; unzip master.zip; rm -f master.zip; sed -i 's@ROOT_DIR=.*@ROOT_DIR='"`pwd | awk -F/ 'NF{NF-=1};1' | sed --expression='s@ @/@g'`"'@' PBMake-master/Makefile.inc; mv PBMake-master ../PBMake; fi

# Makefile definitions
MAKEFILE_INC=../PBMake/Makefile.inc
include $(MAKEFILE_INC)

# Rules to make the executable
main: \
		main.o \
		$(genalg_EXE_DEP) \
		$(genalg_DEP)
	$(COMPILER) `echo "$(genalg_EXE_DEP) main.o" | tr ' ' '\n' | sort -u` $(LINK_ARG) $(genalg_LINK_ARG) -o main 
	
main.o: \
		main.c \
		$(genalg_INC_H_EXE) \
		$(genalg_EXE_DEP)
	$(COMPILER) $(BUILD_ARG) $(genalg_BUILD_ARG) `echo "$(genalg_INC_DIR)" | tr ' ' '\n' | sort -u` -c main.c
	
