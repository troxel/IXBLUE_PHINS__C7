#///////////////////
# Compiler and flags
CC=gcc
CFLAGS=-g -Wall -I ./inc

LIBS=-lm -lpthread

# Directories
OBJDIR=./obj
BINDIR=./bin
INCDIR=./bin

# Create directories if they don't exist
$(shell mkdir -p $(OBJDIR))
$(shell mkdir -p $(BINDIR))

# Common source files
COMMON_SRC=udp_util.c phins_util.c
COMMON_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(COMMON_SRC))

# Target 1 specific files
TARGET1=read_stdbin.exe
TARGET1_SRC=read_stdbin.c haversine.c stbn_util.c
TARGET1_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET1_SRC))

# Target 2 specific files
TARGET2=write_phins.exe
TARGET2_SRC=write_phins.c
TARGET2_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET2_SRC))

# Target 3 specific files 
TARGET3=write_phins_cmds.exe
TARGET3_SRC=write_phins_cmds.c 
TARGET3_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET3_SRC))

# Target 4 specific files 
TARGET4=getphins_data.exe
TARGET4_SRC=getphins_data.c shm_util.c
TARGET4_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET4_SRC))

# Target 5 specific files 
TARGET5=write_dpth_phins.exe
TARGET5_SRC=write_dpth_phins.c 
TARGET5_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET5_SRC))

# Target 6 specific files 
TARGET6=data2csv.exe
TARGET6_SRC=data2csv.c
TARGET6_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET6_SRC))

# Target 7 specific files 
TARGET7=write_atacs.exe
TARGET7_SRC=write_atacs.c haversine.c
TARGET7_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET7_SRC))

# Target 8 specific files (special process for passing thru GPS data for displacement test)
TARGET8=write_gps.exe
TARGET8_SRC=write_gps.c haversine.c
TARGET8_OBJ=$(patsubst %.c,$(OBJDIR)/%.o,$(TARGET8_SRC))

# Targets
all: $(BINDIR)/$(TARGET1) $(BINDIR)/$(TARGET2) $(BINDIR)/$(TARGET3) $(BINDIR)/$(TARGET4) $(BINDIR)/$(TARGET5) $(BINDIR)/$(TARGET6) $(BINDIR)/$(TARGET7) $(BINDIR)/$(TARGET8)

$(BINDIR)/$(TARGET1): $(COMMON_OBJ) $(TARGET1_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET1_OBJ) -Wall $(LIBS) -o $@ 

$(BINDIR)/$(TARGET2): $(COMMON_OBJ) $(TARGET2_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET2_OBJ) -Wall ${LIBS} -o $@

$(BINDIR)/$(TARGET3): $(COMMON_OBJ) $(TARGET3_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET3_OBJ) -Wall ${LIBS} -o $@

$(BINDIR)/$(TARGET4): $(COMMON_OBJ) $(TARGET4_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET4_OBJ) -Wall ${LIBS} -o $@

$(BINDIR)/$(TARGET5): $(COMMON_OBJ) $(TARGET5_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET5_OBJ) -Wall ${LIBS} -o $@

$(BINDIR)/$(TARGET6): $(COMMON_OBJ) $(TARGET6_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET6_OBJ) -Wall ${LIBS} -o $@

$(BINDIR)/$(TARGET7): $(COMMON_OBJ) $(TARGET7_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET7_OBJ) -Wall ${LIBS} -o $@

$(BINDIR)/$(TARGET8): $(COMMON_OBJ) $(TARGET8_OBJ)
	$(CC) $(COMMON_OBJ) $(TARGET8_OBJ) -Wall ${LIBS} -o $@

$(OBJDIR)/%.o: %.c
	$(CC)  -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJDIR)/*.o $(BINDIR)/$(TARGET1) $(BINDIR)/$(TARGET2) $(BINDIR)/$(TARGET3) $(BINDIR)/$(TARGET4) $(BINDIR)/$(TARGET5) $(BINDIR)/$(TARGET6) $(BINDIR)/$(TARGET7) $(BINDIR)/$(TARGET8)

.PHONY: all clean