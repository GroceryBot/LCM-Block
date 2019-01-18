include ../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
CXXFLAGS = $(CXXFLAGS_STD) $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_RPLIDAR)
LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_LCM_TUTORIAL = $(BIN_PATH)/lcm_tutorial


ALL = $(BIN_LCM_TUTORIAL)

all: $(ALL)

$(BIN_LCM_TUTORIAL): lcm_tutorial.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
