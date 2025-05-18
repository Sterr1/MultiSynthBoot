
/* main.cpp */
#include "circle_stdlib_app.h"
#include "main.h"

#define MLOGERR(...) CLogger::Get()->Write("MultiSynth", LogNotice, __VA_ARGS__)
#define MLOGNOTE(...) CLogger::Get()->Write("MultiSynth", LogNotice, __VA_ARGS__)
#define DelayMs(x) m_Timer.MsDelay(x)

extern "C" void start_synth(const char* name); // implemented in each synth's kernel.cpp

CMultiSynthLoader::CMultiSynthLoader()
    : CStdlibAppStdio ("MultiSynth","sdmc"),
      m_LCD(nullptr),
      m_pLCDBuffered(nullptr),
      m_Interrupt(),
      m_Timer(&m_Interrupt),
      m_GPIOManager(&m_Interrupt),
      m_I2CMaster(CMachineInfo::Get ()->GetDevice (DeviceI2CMaster), TRUE),
      m_SPIMaster(nullptr),
      m_Serial(&m_Interrupt, &m_Timer),
      m_Logger(LogDebug, &m_Timer, TRUE),
      m_pUSBMIDIDevice(nullptr),
      m_pMiniDexedConfig(nullptr),
      m_pConfig(nullptr),
      m_bShouldStartSynth(false)
    {

    }

bool CMultiSynthLoader::Initialize()
    {
        if (!CStdlibAppStdio::Initialize ())
	    {
		return FALSE;
	    }
    
        if (!m_Timer.Initialize() ||
            !m_GPIOManager.Initialize() ||
            !m_I2CMaster.Initialize() ||
            !m_SPIMaster->Initialize())
        {
            return FALSE;
        }

    	//const char* items[SYNTH_ITEM_COUNT] = {"MiniDexed", "MiniJV880", "MT-32Pi"};
    	//int selected = 0;
    
        FRESULT res = f_mount(&m_FileSystem, "0:", 1);  // "0:" - номер тома
    if (res != FR_OK) {
        m_Logger.Write(GetKernelName(), LogError, "Mount failed: %d", res);
        return false;
    }
    
    m_pConfig = new CPropertiesFatFsFile("synth.ini", &m_FileSystem);
    if (!m_pConfig->Load()) {
        MLOGERR("Failed to load synth.ini");
        return FALSE;
    }

    m_pMiniDexedConfig = new CPropertiesFatFsFile("minidexed.ini", &m_FileSystem);
    if (!m_pMiniDexedConfig->Load()) {
        MLOGERR("Failed to load minidexed.ini");
        return FALSE;
    }

	unsigned nSPIMaster = m_pMiniDexedConfig->GetNumber("SPIBus()",SPI_INACTIVE);
	unsigned nSPIMode = m_pMiniDexedConfig->GetNumber("SPIMode", SPI_DEF_MODE);
	unsigned long nSPIClock = 1000 * m_pMiniDexedConfig->GetNumber("SPIClockKHz", SPI_DEF_CLOCK);

#if RASPPI<4
	// By default older RPI versions use SPI 0.
	// It is possible to build circle to support SPI 1 for
	// devices that use the 40-pin header, but that isn't
	// enabled at present...
	if (nSPIMaster == 0)
#else
	// RPI 4+ has several possible SPI Bus Configurations.
	// As mentioned above, SPI 1 is not built by default.
	// See circle/include/circle/spimaster.h
	if (nSPIMaster == 0 || nSPIMaster == 3 || nSPIMaster == 4 || nSPIMaster == 5 || nSPIMaster == 6)
#endif
	{
		unsigned nCPHA = (nSPIMode & 1) ? 1 : 0;
		unsigned nCPOL = (nSPIMode & 2) ? 1 : 0;
		m_SPIMaster = new CSPIMaster (nSPIClock, nCPOL, nCPHA, nSPIMaster);
		if (!m_SPIMaster->Initialize())
		{
			delete (m_SPIMaster);
			m_SPIMaster = nullptr;
		}
	}

    //unsigned synth = m_pConfig->GetNumber("synth", 0);

	m_LCDColumns = m_pMiniDexedConfig->GetNumber("LCDColumns", 16);
    m_LCDRows = m_pMiniDexedConfig->GetNumber("LCDRows", 2);

	if (m_pMiniDexedConfig->GetNumber("EncoderEnabled", 0))
	
	{
		m_pRotaryEncoder = new CKY040 (m_pMiniDexedConfig->GetNumber("EncoderPinClock", 10),
					       m_pMiniDexedConfig->GetNumber("EncoderPinData", 9),
					       m_pMiniDexedConfig->GetNumber("GetButtonPinShortcut", 11),
					       &m_GPIOManager);
		if (!m_pRotaryEncoder->Initialize ())
		{
			return false;
		}
		if (m_pRotaryEncoder)
        {
            m_pRotaryEncoder->RegisterEventHandler(EncoderEventStub, this);
        }
		MLOGNOTE ("Rotary encoder initialized");
	} 

    m_PinLeft.AssignPin(m_pMiniDexedConfig->GetNumber("ButtonPinPrev", 5));
    m_PinLeft.SetMode(GPIOModeInput, true);
    m_PinLeft.SetPullMode(GPIOPullModeUp);

    m_PinRight.AssignPin(m_pMiniDexedConfig->GetNumber("ButtonPinNext", 6));
    m_PinRight.SetMode(GPIOModeInput, true);
    m_PinRight.SetPullMode(GPIOPullModeUp);

    m_PinSelect.AssignPin(m_pMiniDexedConfig->GetNumber("ButtonPinSelect", 13));
    m_PinSelect.SetMode(GPIOModeInput, true);
    m_PinSelect.SetPullMode(GPIOPullModeUp);

	if (!m_Serial.Initialize(m_pMiniDexedConfig->GetNumber("MIDIBaudRate", 31250)))
{
    MLOGERR("\nSerial MIDI init failed!");
    m_Timer.MsDelay(2000);
    return FALSE;
}
	unsigned ser_options = m_Serial.GetOptions();
	ser_options &= ~(SERIAL_OPTION_ONLCR);
	m_Serial.SetOptions(ser_options);

	// Load MIDI control mappings from config
    m_midiNext = m_pMiniDexedConfig->GetNumber("MIDIButtonNext", 47);
    m_midiPrev = m_pMiniDexedConfig->GetNumber("MIDIButtonPrev", 46);
    m_midiSelect = m_pMiniDexedConfig->GetNumber("MIDIButtonSelect", 49);
    m_midiNotes[0] = m_pConfig->GetNumber("MIDINote1", 36);
    m_midiNotes[1] = m_pConfig->GetNumber("MIDINote2", 38);
    m_midiNotes[2] = m_pConfig->GetNumber("MIDINote3", 40);

    m_pUSBMIDIDevice = nullptr;
    m_bUSBMIDIInitialized = false;

    m_pUSB = new CUSBHCIDevice (&mInterrupt, &mTimer, TRUE);
	if (!m_pUSB->Initialize ())
	{
		return FALSE;
	}

    /*m_pUSBDevice = new CUSBDevice(m_pUSB);
    if (!m_pUSBDevice->Initialize())
    {
        MLOGERR("USB device init failed");
        delete m_pUSB;
        return FALSE;
    }*/

	if (!m_LCD){
		MLOGERR("Failed initialize LCD");
		return FALSE;
	}

        return TRUE;
    }

CStdlibApp::TShutdownMode CMultiSynthLoader::Run()
{
    m_bShouldStartSynth = false;
    m_SelectedSynth = 0;
	const char* synths[SYNTH_ITEM_COUNT] = {"minidexed", "minijv880", "mt32pi"};
    UpdateDisplay();
        
    while (true)
        {
        if (!m_bUSBMIDIInitialized)
        {
            CheckUSBMIDI();
        }

        if (m_PinRight.Read() == LOW) {
        DelayMs(200); // Простейшая антидребезговая задержка
        m_SelectedSynth = (m_SelectedSynth + 1) % SYNTH_ITEM_COUNT;
        UpdateDisplay();
    }
    else if (m_PinLeft.Read() == LOW) {
        DelayMs(200);
        m_SelectedSynth = (m_SelectedSynth + SYNTH_ITEM_COUNT - 1) % SYNTH_ITEM_COUNT;
        UpdateDisplay();
    }
    else if (m_PinSelect.Read() == LOW) {
        DelayMs(200);
        m_bShouldStartSynth = true;
    }
            
        if (m_bShouldStartSynth) {
            Deinit();
            start_synth(synths[m_SelectedSynth]);
            return ShutdownReboot;
            //return; 
        }

            ProcessMIDIInput();
            m_Timer.MsDelay(10);
        }
        
}

void HandleEncoderEvent(CKY040::TEvent Event)
    {
        CMultiSynthLoader* pThis = static_cast<CMultiSynthLoader*>;

    switch (Event)
        {
        case CKY040::EventClockwise:
            m_SelectedSynth = (m_SelectedSynth + 1) % SYNTH_ITEM_COUNT;
            UpdateDisplay();
            break;

        case CKY040::EventCounterclockwise:
            m_SelectedSynth = (m_SelectedSynth + SYNTH_ITEM_COUNT - 1) % SYNTH_ITEM_COUNT;
            UpdateDisplay();
            break;

        case CKY040::EventSwitchClick:
            m_bShouldStartSynth = true;
            break;

        default:
            break;
        }
    }
	
static void EncoderEventStub(CKY040::TEvent Event, void* pParam)
{
    CMultiSynthLoader* pThis = static_cast<CMultiSynthLoader*>(pParam);
    pThis->HandleEncoderEvent(Event);
}


bool CMultiSynthLoader::LCDinit() 
    {
		unsigned i2caddr = m_pMiniDexedConfig->GetNumber("LCDI2CAddress", 0);
		unsigned ssd1306addr = m_pMiniDexedConfig->GetNumber("SSD1306LCDI2CAddress", 0x3c);
		bool st7789 = m_pMiniDexedConfig->GetST7789Enabled ();
		if (ssd1306addr != 0) {
			m_pSSD1306 = new CSSD1306Device (m_pMiniDexedConfig->GetNumber("SSD1306LCDWidth", 128), 
											 m_pMiniDexedConfig->GetNumber("SSD1306LCDHeight", 32),
											 m_pI2CMaster, ssd1306addr,
											 m_pMiniDexedConfig->GetNumber("SSD1306LCDRotate", 0), 
											 m_pMiniDexedConfig->GetNumber("SSD1306LCDMirror", 0));
			if (!m_pSSD1306->Initialize ())
			{
				MLOGNOTE("LCD: SSD1306 initialization failed");
				return false;
			}
			MLOGNOTE ("LCD: SSD1306");
			m_LCD = m_pSSD1306;
		}
		else if (st7789)
		{
			if (m_pSPIMaster == nullptr)
			{
				MLOGNOTE("LCD: ST7789 Enabled but SPI Initialisation Failed");
				return false;
			}

			unsigned long nSPIClock = 1000 * m_pMiniDexedConfig->GetNumber("SPIClockKHz", SPI_DEF_CLOCK);
			unsigned nSPIMode = m_pMiniDexedConfig->GetNumber("SPIMode", SPI_DEF_MODE);
			unsigned nCPHA = (nSPIMode & 1) ? 1 : 0;
			unsigned nCPOL = (nSPIMode & 2) ? 1 : 0;
			MLOGNOTE("SPI: CPOL=%u; CPHA=%u; CLK=%u",nCPOL,nCPHA,nSPIClock);
			m_pST7789Display = new CST7789Display (m_pSPIMaster,
							m_pMiniDexedConfig->GetNumber("ST7789Data", 0),
							m_pMiniDexedConfig->GetNumber("ST7789Reset", 0),
							m_pMiniDexedConfig->GetNumber("ST7789Backlight", 0 ),
							m_pMiniDexedConfig->GetNumber("ST7789Width", 240),
							m_pMiniDexedConfig->GetNumber("ST7789Height", 240),
							nCPOL, nCPHA, nSPIClock,
							m_pMiniDexedConfig->GetNumber("ST7789Select", 0));
			if (m_pST7789Display->Initialize())
			{
				m_pST7789Display->SetRotation (m_pMiniDexedConfig->GetNumber("ST7789Rotation", 0));
				bool bLargeFont = !(m_pMiniDexedConfig->GetNumber("ST7789SmallFont", 0));
				m_pST7789 = new CST7789Device (m_pSPIMaster, m_pST7789Display, m_pMiniDexedConfig->GetNumber("LCDColumns", 0), m_pMiniDexedConfig->GetNumber("LCDRows", 0), Font8x16, bLargeFont, bLargeFont);
				if (m_pST7789->Initialize())
				{
					MLOGNOTE ("LCD: ST7789");
					m_LCD = m_pST7789;
				}
				else
				{
					MLOGNOTE ("LCD: Failed to initalize ST7789 character device");
					delete (m_pST7789);
					delete (m_pST7789Display);
					m_pST7789 = nullptr;
					m_pST7789Display = nullptr;
					return false;
				}
			}
			else
			{
				MLOGNOTE ("LCD: Failed to initialize ST7789 display");
				delete (m_pST7789Display);
				m_pST7789Display = nullptr;
				return false;
			}
		}
		else if (i2caddr == 0)
		{
			m_pHD44780 = new CHD44780Device (m_pMiniDexedConfig->GetNumber("LCDColumns", 16), 
												m_pMiniDexedConfig->GetNumber("LCDRows", 2),
												m_pMiniDexedConfig->GetNumber("LCDPinData4", 22),
												m_pMiniDexedConfig->GetNumber("LCDPinData5", 23),
												m_pMiniDexedConfig->GetNumber("LCDPinData6",24 ),
												m_pMiniDexedConfig->GetNumber("LCDPinData7", 25),
												m_pMiniDexedConfig->GetNumber("LCDPinEnable", 4),
												m_pMiniDexedConfig->GetNumber("LCDPinRegisterSelect", 27),
												m_pMiniDexedConfig->GetNumber("LCDPinReadWrite", 0));
			if (!m_pHD44780->Initialize ())
			{
				MLOGNOTE("LCD: HD44780 initialization failed");
				return false;
			}
			MLOGNOTE ("LCD: HD44780");
			m_LCD = m_pHD44780;
		}
		else
		{
			m_pHD44780 = new CHD44780Device (m_pI2CMaster, i2caddr,
							m_pMiniDexedConfig->GetNumber("LCDColumns", 0), m_pMiniDexedConfig->GetNumber("LCDRows", 0));
			if (!m_pHD44780->Initialize ())
			{
				MLOGNOTE("LCD: HD44780 (I2C) initialization failed");
				return false;
			}
			MLOGNOTE ("LCD: HD44780 I2C");
			m_LCD = m_pHD44780;
		}
		assert (m_LCD);

		m_pLCDBuffered = new CWriteBufferDevice (m_LCD);
		assert (m_LCDBuffered);
		// clear sceen and go to top left corner
		LCDWrite ("\x1B[H\x1B[J");		// cursor home and clear screen
		LCDWrite ("\x1B[?25l\x1B""d+");		// cursor off, autopage mode
		LCDWrite ("Multisynth\nLoading...");
		m_pLCDBuffered->Update ();

		MLOGNOTE ("LCD initialized");
        return true;
	}

};

void CMultiSynthLoader::LCDWrite (const char *pString)
{
	if (m_LCD)
	{
		m_LCD->Write (pString, strlen (pString));
	}
}

void CMultiSynthLoader::UpdateDisplay() 
{
    if (!m_LCD || !m_pLCDBuffered) return;
    
    const char* synthNames[SYNTH_ITEM_COUNT] = {"MiniDexed", "MiniJV880", "MT-32Pi"};
    const char* currentName = synthNames[m_SelectedSynth];
    
    // Формируем строку вывода
    char displayLine[32] = {0}; // Буфер с запасом
    snprintf(displayLine, sizeof(displayLine), "%s %s %s",
             (m_SelectedSynth > 0) ? "<" : " ",
             currentName,
             (m_SelectedSynth < SYNTH_ITEM_COUNT-1) ? ">" : " ");
    
    // Очистка и вывод
    LCDWrite("\x1B[H\x1B[J"); // Clear screen
    LCDWrite("\x1B[?25l");    // Hide cursor
    LCDWrite("Select Synth\n");
    LCDWrite(displayLine);
    
    m_pLCDBuffered->Update();
}

bool CMultiSynthLoader::CheckUSBMIDI()
{
    if (CDeviceNameService::Get()->GetDevice("umidi1", FALSE) == nullptr)
    {
        MLOGNOTE("USB MIDI device not found yet...");
        return false;
    }

    m_pUSBMIDIDevice = (CUSBMIDIDevice*)CDeviceNameService::Get()->GetDevice("umidi1", FALSE);
    if (m_pUSBMIDIDevice)
    {
        m_pUSBMIDIDevice->RegisterPacketHandler(CUSBMIDIDevice::TMIDIPacketHandlerEx(MIDIPacketHandler), this);
        m_bUSBMIDIInitialized = true;
        MLOGNOTE("USB MIDI device registered");
        return true;
    }

    return false;
}

void CMultiSynthLoader::ProcessMIDIInput()
{
    u8 serialBuffer[64];
    int serialBytes = m_Serial.Read(serialBuffer, sizeof(serialBuffer));
    ProcessRawMIDIData(serialBuffer, serialBytes);  // Общая обработка

}

void CMultiSynthLoader::MIDIPacketHandler(unsigned nCable, u8 *pPacket, 
                                        unsigned nLength, void *pParam)
{
    CMultiSynthLoader* pThis = static_cast<CMultiSynthLoader*>(pParam);
    pThis->HandleMIDIPacket(pPacket, nLength);
}

void CMultiSynthLoader::HandleMIDIPacket(u8 *pPacket, unsigned nLength)
{
    if (nLength < 1) return;

    const u8 status = pPacket[0] & 0xF0;
    //const u8 channel = pPacket[0] & 0x0F;

    // Control Change
    if (status == 0xB0 && nLength >= 3)
    {
        if (pPacket[2] > 0) // Only on non-zero values
        {
            if (pPacket[1] == m_midiNext)
            {
                m_SelectedSynth = (m_SelectedSynth + 1) % SYNTH_ITEM_COUNT;
                UpdateDisplay();
            }
            else if (pPacket[1] == m_midiPrev)
            {
                m_SelectedSynth = (m_SelectedSynth + SYNTH_ITEM_COUNT - 1) % SYNTH_ITEM_COUNT;
                UpdateDisplay();
            }
            else if (pPacket[1] == m_midiSelect)
            {
                m_bShouldStartSynth = true;
            }
        }
    }
    // Note On
    else if (status == 0x90 && nLength >= 3 && pPacket[2] > 0)
    {
        for (int i = 0; i < SYNTH_ITEM_COUNT; i++)
        {
            if (pPacket[1] == m_midiNotes[i])
            {
                m_SelectedSynth = i;
                m_bShouldStartSynth = true;
                break;
            }
        }
    }
    // System Real-Time Messages (просто пропускаем)
    else if (status == 0xF8)
    {
        
    }
    // Active Sensing
    else if (status == 0xFE)
    {
        
    }
    // System Exclusive (начало)
    else if (pPacket[0] == 0xF0)
    {
        
    }
    // System Exclusive (окончание)
    else if (pPacket[0] == 0xF7)
    {
        
    }
}

void CMultiSynthLoader::ProcessRawMIDIData(u8* pData, int nBytes)
{
    if (nBytes <= 0 || !pData) return;

    // MIDI parsing state
    static u8 status = 0;
    static int msgIndex = 0;
    static u8 msg[3];
    static int msgLen = 3; // Default length

    for (int i = 0; i < nBytes; ++i)
    {
        u8 byte = pData[i];

        // Status byte handling
        if (byte & 0x80)
        {
            status = byte;
            int msgIndex = 0;
            msgLen = ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) ? 2 : 3;
        }

        // Store data bytes
        if (msgIndex < sizeof(msg))
            msg[msgIndex++] = byte;

        // Process complete message
        if (msgIndex >= msgLen)
        {
            const u8 type = status & 0xF0;
            
            // Control Change
            if (type == 0xB0 && msg[2] > 0) // CC with value > 0
            {
                if (msg[1] == m_midiNext)
                {
                    m_SelectedSynth = (m_SelectedSynth + 1) % SYNTH_ITEM_COUNT;
                    UpdateDisplay();
                }
                else if (msg[1] == m_midiPrev)
                {
                    m_SelectedSynth = (m_SelectedSynth + SYNTH_ITEM_COUNT - 1) % SYNTH_ITEM_COUNT;
                    UpdateDisplay();
                }
                else if (msg[1] == m_midiSelect)
                {
                    m_bShouldStartSynth = true;
                }
            }
            // Note On
            else if (type == 0x90 && msg[2] > 0)
            {
                for (int i = 0; i < SYNTH_ITEM_COUNT; i++)
                {
                    if (msg[1] == m_midiNotes[i])
                    {
                        m_SelectedSynth = i;
                        m_bShouldStartSynth = true;
                        break;
                    }
                }
            }

            msgIndex = 0; // Reset for next message
        }
    }
}

void CMultiSynthLoader::Deinit()
    {
        delete m_pSSD1306;
        delete m_pST7789;
        delete m_pST7789Display;
        delete m_pHD44780;
        delete m_pLCDBuffered; 
        delete m_pMiniDexedConfig;
        delete m_pConfig;
        delete m_pRotaryEncoder;
        delete m_SPIMaster;
        f_unmount("0:");
    }

extern "C" void start_synth(const char* name) {
    static int synth_counter = 0;
    /*CMultiSynthLoader kernel;
    
    // Используем логгер через экземпляр класса
    kernel.m_Logger.Write("MultiSynth", LogNotice, 
                         "Starting synth: %s (call #%d)", 
                         name, ++synth_counter);

    while (true) {
        kernel.ProcessMIDIInput();
        kernel.m_Timer.MsDelay(10);
        
        if (kernel.ReadButton(kernel.m_PinSelect)) {
            break;
        }
    }
    */
    MLOGNOTE("Synth started");
}
MULTI_CORE_APPLICATION(CMultiSynthLoader);