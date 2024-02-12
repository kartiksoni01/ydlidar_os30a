MAKEFILE = Makefile

ifeq ($(CPU),X86)
	TARGET = test_x86
	LIBS = -L../eSPDI -leSPDI_X86_64 \
	       -lrt -lm -ldl -lpthread -lX11 -ludev
endif
ifeq ($(CPU), NVIDIA)
	TARGET = test_arm64_tx2
	LIBS = -L../eSPDI -leSPDI_$(CPU)_64 \
	       -lrt
endif
ifeq ($(CPU), PX30)
	TARGET = test_arm64_px30
	LIBS = -L../eSPDI -leSPDI_$(CPU)_64 \
	       -lrt
endif
ifeq ($(CPU), MTK)
	TARGET = test_arm64_mtk
	LIBS = -L../eSPDI -leSPDI_$(CPU)_64 \
	       -lrt
endif
ifeq ($(CPU), TI)
	TARGET = test_armhf_ti
	LIBS = -L../eSPDI -leSPDI_$(CPU)_32 \
	       -lrt
endif

CFLAGS = -Wall -W -std=c++11
LFLAGS = -Wall -W 
LIBS += -lpthread
OBJECTS = main.o ColorPaletteGenerator.o RegisterSettings.o PlyWriter.o
INCPATH = -I../eSPDI

####### Implicit rules

.SUFFIXES: .o .cpp

.cpp.o:
	$(CXX) -c $(CFLAGS) $(INCPATH) -g -o $@ $<

all: $(TARGET)

####### Build rules
$(TARGET): $(OBJECTS)
	$(CXX) $(LFLAGS) -g -o $@ $^ $(LIBS)

####### clean rules
clean: 
	rm $(OBJECTS) $(TARGET)
distclean:
	rm $(OBJECTS)
