// kernel.h
#pragma once

#include "circle_stdlib_app.h"
#include <circle/startup.h>
#include <circle/types.h>
#include <circle/memio.h>
#include <circle/interrupt.h>
#include <circle/gpiomanager.h>
#include <circle/gpiopin.h>
#include <circle/i2cmaster.h>
#include <circle/spimaster.h>
#include <circle/serial.h>
#include <circle/sysconfig.h>
#include <circle/usb/usbhcidevice.h>
#include <circle/usb/usbmidi.h>
#include <display/hd44780device.h>
#include <display/ssd1306device.h>
#include <display/st7789device.h>
#include <display/chardevice.h>
#include <circle/screen.h>
#include <circle/writebuffer.h>
#include <circle/multicore.h>
#include <sensor/ky040.h>
#include <fatfs/ff.h>
#include <Properties/propertiesfatfsfile.h>
#include <cstdio>
#include <array>
#include <stdarg.h>

#define MAX_MIDI_MESSAGE 128
#define SYNTH_ITEM_COUNT 3
#define MULTI_CORE_APPLICATION(className) \
    className Kernel; \
    void kernel_main(void) { Kernel.Run(); }
#define SPI_INACTIVE	255
#define SPI_DEF_CLOCK	15000	// kHz
#define SPI_DEF_MODE	0		// Default mode (0,1,2,3)

class CKernel : public CStdlibAppStdio
{
public:
    CKernel(void);
    virtual ~CKernel(void);
    bool Initialize(void);
    TShutdownMode Run(void);
    bool CheckUSBMIDI(void);
    const char* GetKernelName() const { return "MultiSynth"; }
    bool LCDinit(void);
    void LCDWrite(const char *pString);
    bool ShouldExit() const { return m_bShouldExit; }
    void SetExitFlag(bool flag) { m_bShouldExit = flag; }
    CGPIOPin m_PinSelect;
    CLogger* GetLogger() { return &m_Logger; }
    void HandleEncoderEvent(CKY040::TEvent Event);
 

private:
    static void PanicHandler (void);             
    bool m_bShouldExit = false;
    unsigned m_LCDColumns;
    unsigned m_LCDRows;
    const std::array<const char*, SYNTH_ITEM_COUNT> m_SynthNames = {"MiniDexed", "MiniJV880", "MT-32Pi"};

private:
    CScreenDevice* m_pScreen;
    CCharDevice *m_LCD;
    CWriteBufferDevice *m_pLCDBuffered = nullptr;
    CSSD1306Device* m_pSSD1306 = nullptr;
    CST7789Device* m_pST7789 = nullptr;
    CST7789Display* m_pST7789Display = nullptr;
    CHD44780Device* m_pHD44780 = nullptr;

    CInterruptSystem m_Interrupt;
public:
    CTimer m_Timer;
private:
    CGPIOManager m_GPIOManager;
    CI2CMaster m_I2CMaster;
    CSPIMaster* m_SPIMaster = nullptr;
    CSerialDevice m_Serial;
    CLogger m_Logger;
    CUSBMIDIDevice* m_pUSBMIDIDevice;
    CPropertiesFatFsFile* m_pMiniDexedConfig; 
    CPropertiesFatFsFile* m_pConfig;
protected:
    int m_SelectedSynth = 0;
    bool m_bShouldStartSynth = false;
private:    
    
    CKY040* m_pRotaryEncoder = nullptr;
    FATFS m_FileSystem;
    CUSBHCIDevice* m_pUSB;
    //CUSBDevice* m_pUSBDevice; 
    
    void UpdateDisplay(void);
    void ProcessRawMIDIData(u8* pData, int nBytes);
    void Deinit(void);
    void ProcessMIDIInput(void);
    void HandleMIDIPacket(u8* pPacket, unsigned nLength);
    static void USBMIDIMessageHandler(unsigned nCable, u8 *pPacket,
                                       unsigned nLength);
    static void DeviceRemovedHandler(CDevice *pDevice, void *pContext);
    static void EncoderEventStub(CKY040::TEvent Event, void* pParam) {
        static_cast<CKernel*>(pParam)->HandleEncoderEvent(Event);
    }

    // Button pins
    CGPIOPin m_PinLeft;
    CGPIOPin m_PinRight;
    unsigned m_midiNext = 0;
    unsigned m_midiPrev = 0;
    unsigned m_midiSelect = 0;
    unsigned m_midiNotes[SYNTH_ITEM_COUNT];
    u8  m_MIDIBuffer[MAX_MIDI_MESSAGE];
    bool m_bUSBMIDIInitialized = false;

    static CKernel* s_pThis;
};