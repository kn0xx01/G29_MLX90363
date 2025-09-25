// ---------------------------------------------------------------------------------------------------- //
// Robert's Smorgasbord 2024                                                                            //
// https://robertssmorgasbord.net                                                                       //
// https://www.youtube.com/@robertssmorgasbord                                                          //
// Melexis MLX90363 Triaxis® Magnetometer & Arduino MCU – The Details (15) https://youtu.be/v6Q-92aF9bA //
// ---------------------------------------------------------------------------------------------------- //

// References ([...]) are to datasheet revision 006, December 2016

#ifndef MLX90363_H
#define MLX90363_H

#include <SPI.h>

class MLX90363
{   
   public:

   // Received
   // --------
   // Type of a MISO message received from the MLX90363. Can be retrieved via the received() method.
   // (*) Some types of MISO messages are handled by the library and should never be returned by received().     

   enum class Received : uint8_t
   {  
   // MISO Message     Marker       OpCode   Comment
   // --------------   ----------   ------   --------
      alpha          = 0b00000000,        // Regular alpha + diagnostic message
      alpha_beta     = 0b01000000,        // Regular alpha-beta + diagnostic message
      x_y_z          = 0b10000000,        // Regular x-y-z + diagnostic message
      memory_read    = 0b11000000 | 0x02, // After MOSI memory read message (*)
      ee_challenge   = 0b11000000 | 0x04, // After MOSI EEPROM write message (*)
      ee_status      = 0b11000000 | 0x0E, // After MOSI EEPROM response message (*)
      nop            = 0b11000000 | 0x11, // After MOSI NOP (No OPeration) message
      diagnostic     = 0b11000000 | 0x17, // After MOSI diagnostic message
      osc_cntr_start = 0b11000000 | 0x19, // After MOSI oscillator counter start message (*)
      osc_cntr_stop  = 0b11000000 | 0x1B, // After MOSI oscillator counter stop message (*)
      ee_writing     = 0b11000000 | 0x28, // After MOSI EEPROM requesting message (*)
      ready          = 0b11000000 | 0x2C, // After MOSI reboot message or power on
      get3ready      = 0b11000000 | 0x2D, // After first Get3 MOSI message
      standby        = 0b11000000 | 0x32, // After MOSI standby message
      error          = 0b11000000 | 0x3D, // Error
      ntt            = 0b11000000 | 0x3E  // Nothing to transmit
   };

   // General library errors
   // ----------------------
   // These errors are caused by failed library methods (returning false).
   // They can be retrieved with the lastLibraryError() method.
   enum class LibraryError : uint8_t
   {
      crc,    // CRC error in received MISO message
      opcode, // Unknown operation code in received MISO message
      ee_key,
      ee_address,
      ee_erase_write,
      ee_crc,
      ee_challenge
   };

   // Error codes in MISO error messages
   // ----------------------------------
   // These error codes are contained in MISO error messages.
   // They can be retrieved with the lastErrorErrorCode() method.
   enum class ErrorCode : uint8_t
   {
      bitcount = 1, // Incorrect bit count
      crc      = 2, // Incorrect CRC
      ntt      = 3, // Nothing to transmit
      opcode   = 4  // Invalid OpCode
   };

   // Status of Diagnostics (E1, E0 bits) in MISO alpha, alpha-beta and x-y-z messages [13.4]
   // ---------------------------------------------------------------------------------------
   // These diagnostic codes are contained in MISO alpha, alpha-beta and x-y-z messages messages.
   // They can be retrieved with the lastGetDiagnostic() method.
   enum class Diagnostic : uint8_t
   {                            // Description [13.4, Table 10, Diagnostics Status Bits]
      unfinished        = 0b00, // "First Diagnostics Sequence Not Yet Finished"
      failed            = 0b01, // "Diagnostic Fail"
      passed_previously = 0b10, // "Diagnostic Pass (Previous cycle)"
      passed_currently  = 0b11  // "Diagnostic Pass – New Cycle Completed"
   };

   // MapXYZ
   // ------
   // User Configuration: Device Orientation [16.1]
   enum class MapXYZ : uint8_t
   {
   // x  y  z                   B1  B2  B3  Remark
      b1_b2_b3 = 0b00000000, // X   Y   Z   Default
      b1_b3_b2 = 0b00000001, // X   Z   Y
      b3_b1_b2 = 0b00000010, // Y   Z   X
      b2_b1_b3 = 0b00000011, // Y   X   Z   Use b1_b2_b3 instead
      b2_b3_b1 = 0b00000100, // Z   X   Y
      b3_b2_b1 = 0b00000101  // Z   Y   X
   };

   // OrthSel (Orthogonality [Correction] Selection)
   // ----------------------------------------------
   // Select orthogonality correction via OrthXY (3D irrelevant) or OrthB1B2 (3D must be false?????) [18.2].
   enum class OrthSel : uint8_t
   {
      xy   = 0b0,
      b1b2 = 0b1
   };
   
   // SelSMism (Select Sensitivity Mismatch)
   // --------------------------------------
   // Magnetic Angle ∠XY [16.3.1]
   // Magnetic Angle ∠XZ and ∠YZ [16.3.2]: Sensitivity Mismatch between B1 and B2
   // Bx = Bx * SMISM / 2^15 with SMISM the raw 15 bit unsigned integer value
   // Bx = Bx * SMISM        with SMISM a float between 0 and 1 ((2^15 - 1) / 2^15 = 0.999969482421875)
   // Bx either b1 or b2
   // Signal Processing (GETx) [18.2]: Only applicable if 3D is false (0)
   enum class SelSMism : uint8_t
   {
      multiply_b1 = 0b0,
      multiply_b2 = 0b1
   };
  
   // Filter 
   // ------
   // User Configuration: Filter [16.5]
   enum class Filter : uint8_t
   {
      no_filter   = 0b00000000, // (default)
      extra_light = 0b00010000, //
      light       = 0b00100000, //
      medium      = 0b00110000  //
   };

   // PinFilter 
   // ---------
   // EMC Filter on SCI Pins [16.8]
   enum class PinFilter : uint8_t
   {
      spi2mhz   = 0b01, // (default)
      spi1mhz   = 0b10,
      spi500khz = 0b11 
   };

   // Free 
   // ----
   // FREE bytes [16.9]
   struct Free
   {
      uint8_t bytes[5]; 
   };

   // [13.15] [17]
   // ------------
   //
   enum DiagItems : uint32_t
   {                                       // Bit  Description
      ram_march_test     = 0x00000001, // D0   RAM March C-10N Test
      watchdog_self_test = 0x00000002, // D1   Watchdog BIST (Build-In Self-Test)
      rom_checksum       = 0x00000004, // D2   ROM 16 bit Checksum
      ram_test           = 0x00000008, // D3   RAM Test (continuous)
      cpu_register_test  = 0x00000010, // D4   CPU Register Functional Test
      eeprom_parameters  = 0x00000020, // D5   EEPROM Calibration parameters (8 bit CRC)
      eeprom_hamming     = 0x00000040, // D6   EEPROM Hamming Code DED (Dual Error Detection)
      eeprom_ram_cache   = 0x00000080, // D7   EEPROM RAM Cache Error
      adc_block          = 0x00000100, // D8   ADC Block
                         // 0x00000200,    D9   undocumented
                         // 0x00000400,    D10  undocumented
                         // 0x00000800,    D11  undocumented 
      bz_sensitivity     = 0x00001000, // D12  BZ sensitivity monitor
      bx_sensitivity     = 0x00002000, // D13  BX sensitivity monitor
      by_sensitivity     = 0x00004000, // D14  BY sensitivity monitor
      temperature_sensor = 0x00008000, // D15  Temperature sensor monitoring (based on redundancy)
      temperature        = 0x00010000, // D16  Temperature > 190°C or < -80°C (± 20°C) 
      field_high         = 0x00020000, // D17  Field magnitude too high (Norm > 99% ADC Span)
      field_low          = 0x00040000, // D18  Field magnitude too low (Norm < 20% ADC Span)
      adc_clipping       = 0x00080000, // D19  ADC clipping (X, Y, Z, two phases each)
      vdd_vdec_monitors  = 0x00100000  // D20  Supply voltage monitor (VDD) and Regulator monitor (VDEC)
   };
   
   // [13.15]
   // -------
   // Parameter FSMERC [Fail-Safe Mode Entry Root-Cause] reports the root-cause of entry in fail-safe mode [13.15]
   enum class FailSafeCause : uint8_t
   {
      none    = 0b00000000, // "0: the chip is not in fail-safe mode"
      bist    = 0b01000000, // "1: BIST (Build-In Self-Test) error happened and the chip is in fail-safe mode"
      digital = 0b10000000, // "2: digital diagnostic error happened and the chip is in fail-safe mode"
      error   = 0b11000000  // "3: [...] interruptions [...] happened [...] chip is in fail-safe mode"
   };

   // [13.15]
   // -------
   // "If FSMERC = 3, ANADIAGCNT [ANAlog Diagnostic CouNTer] takes another meaning:" 
   // Implementation Note: ANADIAGCNT occupies the 6 LSBs in a byte (0..63). The values below given in the
   //                      datasheet are for the whole byte with FSMERC, which occupies the 2 MSBs in that
   //                      byte, being 3 (0b11000000, respectively, 192 when looking at the whole byte).  
   enum class FailSafeError : uint8_t
   {
      none         = 0b00000000, // Value used when FSMERC is not 3 (FailSafeCause::error)
      protection   = 0b00000001, // "193 protection error interruption happened"
      address      = 0b00000010, // "194 invalid address error interruption happened"
      program      = 0b00000011, // "195 program error interruption happened"
      exchange     = 0b00000100, // "196 exchange error interruption happened"
      connection   = 0b00000101, // "197 not connected error interruption happened"
      stack        = 0b00000110, // "198 Stack Interrupt"
      flow_control = 0b00000111, // "199 Flow Control Error"
   };
   
   // Default Constructor
   // -------------------
   // Is not supported.
   MLX90363() = delete;

   // Constructor
   // -----------
   // SPIClass& spi:        SPIClass object to be used for, SPIClass.begin()
   //                       has to be called before using the MLX90363 object.
   // uint8_t   spi_ss_pin: Slave select pin of the MLX90363 on the SPI bus.
   // uint32t   spi_speed:  Optional SPI speed to be used (default 2000000).
   MLX90363(SPIClass& spi,
            uint8_t   spi_ss_pin,
            uint32_t  spi_speed = spi_speed_max);

   // begin()
   // -------
   // Call before using any other method!
   // SPIClass.begin of the SPIClass object passed in the constructor has to be called first.
   bool begin();

   uint8_t   infoFWVersion();      
   uint8_t   infoHWVersion();
   float     infoOscFreqMHz(uint16_t sample_time_us = 16383);
   uint64_t  infoMLXID();     // Chapter  Parameter       Range      Default   
                              // 14       MLXID           0..48^2-1
                              
   // active...(), update...()
   // ------------------------
   //
                                               // Chapter  Parameter       Range      Default
   MapXYZ    activeMapXYZ();                   // 16.1     MAPXYZ                     x_y_z
   void      updateMapXYZ(MapXYZ to);          //
   bool      active3D();                       // 16.2     3D                         false
   void      update3D(bool to);                //
   OrthSel   activeOrthSel();                  // 16.3     ORTH_SEL                   false
   void      updateOrthSel(OrthSel to);        //
   uint8_t   activeOrthXY();                   // 16.3     ORTH(_XY)       0..8^2-1   preset by Melexis,
                                               //                                     don't change
   uint8_t   activeOrthB1B2Raw();              // 16.3     ORTH_B1B2       0..8^2-1   0
   void      updateOrthB1B2Raw(uint8_t to);    //
   float     activeOrthB1B2();                 // 16.3     ORTH_B1B2       0..2       0
   void      updateOrthB1B2(float to);         //
   SelSMism  activeSelSMism();                 // 16.3     SEL_SMISM
   void      updateSelSMism(SelSMism to);      //
   uint16_t  activeSMismRaw();                 // 16.3     SMISM           0..15^2-1
   void      updateSMismRaw(uint16_t to);      //
   float     activeSMism();                    // 16.3     SMISM           0..1
   void      updateSMism(float to);            //
   uint16_t  activeKAlphaRaw();                // 16.4     KALPHA          0..16^2-1  0
   void      updateKAlphaRaw(uint16_t to);     //
   float     activeKAlpha();                   // 16.4     KALPHA          0..2       0.0
   void      updateKAlpha(float to);           //
   uint16_t  activeKBetaRaw();                 // 16.4     KBETA           0..16^2-1  52428
   void      updateKBetaRaw(uint16_t to);      //
   float     activeKBeta();                    // 16.4     KBETA           0..2       1.6
   void      updateKBeta(float to);            //
   uint16_t  activeKTRaw();                    // 16.4     KT              0..16^2-1  32768
   void      updateKTRaw(uint16_t to);         //
   float     activeKT();                       // 16.4     KT              0..2       1.0000152
   void      updateKT(float to);               //
   Filter    activeFilter();                   // 16.4     FILTER                     no_filter
   void      updateFilter(Filter to);          //
   uint8_t   activeVirtualGainMin();           // 16.6     VIRTUALGAINMIN  0..8^2-1   0
   void      updateVirtualGainMin(uint8_t to); //
   uint8_t   activeVirtualGainMax();           // 16.6     VIRTUALGAINMAX  0..8^2-1   41
   void      updateVirtualGainMax(uint8_t to); //
   uint8_t   activeFHystRaw();                 // 16.7     FHYST           0..8^2-1
   void      updateFHystRaw(uint8_t to);       //
   float     activeFHyst();                    // 16.7     FHYST           0..11.22
   void      updateFHyst(float to);            //
   PinFilter activePinFilter();                // 16.8     PINFILTER                  spi2mhz
   void      updatePinFilter(PinFilter to);    //
   uint32_t  activeUserID();                   // 16.9     USERID          0..2^32-1  0x00010003
   void      updateUserID(uint32_t to);        //
   Free      activeFree();                     // 16.9     FREE                       0, 0, 0, 0, 0
   void      updateFree(Free& to);             //

   // commitConfig()
   // --------------
   // Writes an updated configuration (update...() methods, see above) to the MLX90363's EEPROM. If the
   // configuration indeed changed, the MLX90363 is rebooted, so it reads the new config from its EEPROM,
   // and the active configuration is read back from the MLX90363's EEPROM (active...() methods, see above).
   // Returns true is successfull, otherwise false (use lastLibraryError() to retrive fault condition.
   bool commitConfig();

   // send...()
   // ---------
   // These methods send MOSI messages to the MLX90363 via the SPI bus. They return true is successfull,
   // otherwise false (use lastLibraryError() to retrive fault condition.
   // They also receive a MISO message from the MLX90363. The content of that (last) received message is
   // available through the last...() methods.
   
   // sendGet...()
   // ------------
   // Sends a Get1 (...1...), Get2 (...2...) or Get3 (...3...) MOSI message (trigger modes 1, 2 or 3),
   // for alpha (...Alpha), alpha-beta (...AlphaBeta) or x-y-z (...XYZ) data.
   bool sendGet1Alpha(bool     reset_roll_counter = false,
                      uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet2Alpha(bool     reset_roll_counter = false,
                      uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet3Alpha(bool     reset_roll_counter = false,
                      uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet1AlphaBeta(bool     reset_roll_counter = false,
                          uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet2AlphaBeta(bool     reset_roll_counter = false,
                          uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet3AlphaBeta(bool     reset_roll_counter = false,
                          uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet1XYZ(bool     reset_roll_counter = false,
                    uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet2XYZ(bool     reset_roll_counter = false,
                    uint16_t data_timeout_us    = 0xFFFF );
   bool sendGet3XYZ(bool     reset_roll_counter = false,
                    uint16_t data_timeout_us    = 0xFFFF );

   // sendNOP(), sendReboot(), sendStandby()
   // --------------------------------------
   bool sendNOP(uint16_t key = 0); // Sends a NOP (no operation) MOSI message.
   bool sendReboot();              // Sends a reboot MOSI message.   
   bool sendStandby();             // Sends a standby MOSI message.

   // sendDiagnostic()
   // ----------------
   bool sendDiagnostic(); // Sends a diagnostic MOSI message.

   // received...()
   // -------------
   // These methods provide access to the data contained in the last MISO message received from
   // the MLX90363 during sending a MOSI message to the MLX90363 (see send...() methods).

   // received()
   // ----------
   // Returns the type of the last received MISO message.
   Received received();
   
   // receivedGet...()
   // ----------------
   // Returns the data (diagnostic, roll counter, virtual gain, alpha & beta (raw & in degrees), x & y & z
   // contained in the last received MISO message, if that message was a regular data message.
                                        // Returned value valid only if received MISO message was ...
                                        // alpha  alpha_beta  xyz  range
                                        // -----  ----------  ---  -----------------------
   Diagnostic receivedGetDiagnostic();  //   X        X        X
   uint8_t    receivedGetRollCounter(); //   X        X        X
   uint8_t    receivedGetVirtualGain(); //   X        X
   uint16_t   receivedGetAlphaRaw();    //   X        X
   float      receivedGetAlphaDeg();    //   X        X
   uint16_t   receivedGetBetaRaw();     //            X
   float      receivedGetBetaDeg();     //            X
   int16_t    receivedGetX();           //                     X   -8192 to +8191
   int16_t    receivedGetY();           //                     X   -8192 to +8191
   int16_t    receivedGetZ();           //                     X   -8192 to +8191
   
   // receivedNOPKeyEcho()
   // --------------------
   // Returns the NOP (no operation) echo key contained in the last received MISO message,
   // if the last received MISO message was a NOP message. Otherwise the returned value is undefined.
   uint16_t receivedNOPKeyEcho();

   // receivedNOPInvertedKeyEcho()
   // ----------------------------
   // Returns the NOP (no operation) inverted echo key contained in the last received MISO message,
   // if the last received MISO message was a NOP message. Otherwise the returned value is undefined.
   uint16_t receivedNOPInvertedKeyEcho();

   // receivedDiagnosticDiagItems()
   // -----------------------------
   // Returns the diagnostic items in the last received MISO message,
   // if the last received MISO message was a diagnostic message. Otherwise the returned value is undefined.
   DiagItems receivedDiagnosticDiagItems();
   bool      receivedDiagnosticDiagItems(DiagItems items);

   // receivedDiagnosticFailSafeCause()
   // ---------------------------------
   // Returns the cause why the MLX90363 has entered fail-safe mode contained in the last received MISO message,
   // if the last received MISO message was a diagnostic message. Otherwise the returned value is undefined.
   FailSafeCause receivedDiagnosticFailSafeCause();

   // receivedDiagnosticFailSafeError()
   // ---------------------------------
   // Returns the cause why the MLX90363 has entered fail-safe mode contained in the last received MISO message,
   // if the last received MISO message was a diagnostic message. Otherwise the returned value is undefined.
   FailSafeError receivedDiagnosticFailSafeError();

   // receivedDiagnosticLoopCounter()
   // -------------------------------
   // Returns ... contained in the last received MISO message,
   // Returns -1 if ...
   // if the last received MISO message was a diagnostic message. Otherwise the returned value is undefined.
   int8_t MLX90363::receivedDiagnosticLoopCounter();

   // receivedReadyHWVersion()
   // ------------------------
   // Returns the ready message HW (hardware) version contained in the last received MISO message,
   // if the last received MISO message was a ready message. Otherwise the returned value is undefined.
   uint8_t receivedReadyHWVersion();

   // receivedReadyFWVersion()
   // ------------------------
   // Returns the ready message FW (firmware) version contained in the last received MISO message,
   // if the last received MISO message was a ready message. Otherwise the returned value is undefined.
   uint8_t receivedReadyFWVersion();

   // receivedErrorErrorCode()
   // ------------------------
   // Returns the error message error code contained in the last received MISO message,
   // if the last received MISO message was a error message. Otherwise the returned value is undefined.
   ErrorCode receivedErrorErrorCode();

   // libraryError()
   // --------------
   // Returns the last error encountered by the library.
   LibraryError libraryError();

   private:

   // SPI
   static constexpr uint32_t spi_speed_max         = 2000000;    // Maximum SPI speed in Hz of the MLX90363
   static constexpr uint8_t  message_size          = 8;          // Number of bytes in every message
   // Message Bits
   static constexpr uint8_t  bitmask_marker        = 0b11000000; // Marker in bytes[6]
   static constexpr uint8_t  bitmask_opcode        = 0b00111111; // Opcode in bytes[6]
   static constexpr uint8_t  bitmask_roll_counter  = 0b00111111; // Roll counter in bytes[6]
   static constexpr uint8_t  bitmask_fsmerc        = 0b11000000; // Fail save cause in diagonstic message
   static constexpr uint8_t  bitmask_anadiagcnt    = 0b00111111; // Error interrupt, counter in diagonstic message
   static constexpr uint16_t bitmask_counter_value = 0x7FFF;     // CounterValue in oscillator counter stopped
   static constexpr uint8_t  bitmask_code          = 0b00001111; // Code in bytes[0] of EE status messages
   // Config Bits
   static constexpr uint8_t  bitmask_pinfilter     = 0b00000011; // PINFILTER in Config.values.pinfilter
   static constexpr uint8_t  bitmask_mapxyz        = 0b00000111; // MAPXYZ in Config.values.mapxyz_3d_filter
   static constexpr uint8_t  bitmask_3d            = 0b00001000; // 3D in Config.values.mapxyz_3d_filter
   static constexpr uint8_t  bitmask_orth_sel      = 0b00000001; // ORTH_SEL in Config.values.orth_sel
   static constexpr uint8_t  bitmask_filter        = 0b00110000; // FILTER in Config.values.mapxyz_3d_filter
   static constexpr uint16_t bitmask_sel_smism     = 0x8000;     // SEL_SMISM in Config.values.smism_sel_smism
   static constexpr uint16_t bitmask_smism         = 0x7FFF;     // SMISM in Config.values.smism_sel_smism

   enum class Marker : uint8_t // The first two MSB of the 6th byte (counting from 0) of any
   {                           // 8 byte message. Indicates the (basic) type of message.
      alpha      = 0b00000000, // Regular alpha + diagnostic message
      alpha_beta = 0b01000000, // Regular alpha-beta + diagnostic message
      x_y_z      = 0b10000000, // Regular x-y-z + diagnostic message
      other      = 0b11000000  // Any other non regular message
   };
   
   enum class MOSIOpCode : uint8_t // The last six LSB of the 6th byte (counting from 0) of any
   {                               // 8 byte MOSI message. The OpCode (command) for the MLX90363. 
      memory_read    = 0x01, // read two adresses (2 byte words each) from memory (RAM, EEPROM, ROM)
      ee_write       = 0x03, // write an address (2 byte word) in the EEPROM
      ee_response    = 0x05, // required reply to an MISO EEPROM write challenge message (0x04)
      ee_request     = 0x0F, // next MOSI message after an MOSI EEPROM write message (0x03)
      nop            = 0x10, // no operation
      get1           = 0x13, // get (trigger mode 1) alpha, alpha-beta or x-y-z data (determined by marker)
      get2           = 0x14, // get (trigger mode 2) alpha, alpha-beta or x-y-z data (determined by marker)
      get3           = 0x15, // get (trigger mode 3) alpha, alpha-beta or x-y-z data (determined by marker)
      diagnostic     = 0x16, // diagnostic
      osc_cntr_start = 0x18, // start the counter for the internal oscillator
      osc_cntr_stop  = 0x1A, // stop the counter for the internal oscillator
      reboot         = 0x2F, // only allowed after EEPROM write, in fail-safe, or standby
      standby        = 0x31  // only allowed after NOP (no operation) or diagnostics
   };

   enum class EECode : uint8_t // Code within a EEWriteStatus (OpCode 14d) MISO message [13.7]
   {
      success          = 0b0001, // 1 Success
      erase_write_fail = 0b0010, // 2 Erase/Write Fail
      crc_fail         = 0b0100, // 4 EEPROM CRC Erase/Write Fail
      key_invalid      = 0b0110, // 6 Key Invalid
      challenge_fail   = 0b0111, // 7 Challenge Fail
      odd_address      = 0b1000  // 8 Odd Address
   };

   typedef union MOSIMessage // ATmega is little endian (least significant byte at lowest address)!
   {
      uint8_t bytes[message_size];          // Messages consist of 8 bytes (bytes[0] to bytes[7])
      struct  All                           // All messages contain marker/opcode and CRC
      {
         uint8_t varying[message_size - 2]; // bytes[0] to bytes[5] have varying content
         uint8_t marker_opcode;             // bytes[6] contains the marker and opcode
         uint8_t crc;                       // bytes[7] contains the CRC
      }
      all;
      struct  Get                           // Get1, Get2 or Get3 message (determined by opcode)
      {
         uint8_t  unused;                   // bytes[0] is unused
         uint8_t  reset_roll_counter;       // bytes[1] LSB resets the roll counter
         uint16_t data_timeout_us;          // bytes[2] and bytes[3] contain a timeout in us (max. 65535us),
      }                                     // lifetime of requested data (determined by marker)
      get;
      struct  MemoryRead                    // Read memory message (read two 16 bit words from two addresses)
      {                                     // RAM 0x0000 - 0x00FE, EEPROM 0x1000 - 0x103E, ROM 0x4000 - 0x5FFE
         uint16_t addr0;                    // bytes[0] and bytes[1] are 1st address to be read
         uint16_t addr1;                    // bytes[2] and bytes[3] are 2nd address to be read
      }
      memory_read;
      struct  EEWrite                       // EEPROM write message (write a 16 bit word to an EEPROM address)
      {
         uint8_t  unused;                   // bytes[0] is unused
         uint8_t  address;                  // bytes[1] 6 LSBs are the EEPROM address to be written to
         uint16_t key;                      // bytes[2] and bytes[3] are the key to be provided for the address
         uint16_t data_word;                // bytes[4] and bytes[5] are the data to be written to the EEPROM
      }
      ee_write;
      struct  EEResponse                    // EEPROM response message to a challange message from the slave
      {
         uint16_t unused;                   // bytes[0] and bytes[1] are unused
         uint16_t key_echo;                 // bytes[2] and bytes[3] are the challenge key xor'ed with 0x1234
         uint16_t inverted_key_echo;        // bytes[4] and bytes[5] are the binary inverted key echo
      }
      ee_response;
      struct  NOP                           // No operation message
      {
         uint16_t unused;                   // bytes[0] and bytes[1] are unused 
         uint16_t key;                      // bytes[2] and bytes[3] contain a random value (key)
      }
      nop;

      MOSIMessage()
      : bytes{} // uint8_t bytes[message_size]
      {
      };
      
      MOSIMessage(Marker marker, MOSIOpCode opcode)
      : all{ {0, 0, 0, 0, 0, 0},                  // uint8_t varying[message_size - 2];
             (uint8_t)marker | (uint8_t)opcode,   // uint8_t marker_opcode
             0                                  } // uint8_t crc
      {
      };
   };

   typedef union MISOMessage // ATmega is little endian (least significant byte at lowest address)!
   {
      uint8_t bytes[message_size];          // Messages consist of 8 bytes (bytes[0] to bytes[7])
      struct  All                           // All messages contain the CRC in the last byte
      {
         uint8_t varying[message_size - 2]; // bytes[0] to bytes[5] have varying content
         uint8_t marker;                    // bytes[6] contains the marker
         uint8_t crc;                       // bytes[7] contains the CRC
      }
      all;
      struct  Regular
      {
         uint8_t varying_1;                 // bytes[0] has varying content
         uint8_t diagnostic;                // bytes[1] two MSBs contain the diagnostics
         uint8_t varying_2[4];              // bytes[2] to bytes[5] have varying content
         uint8_t marker_roll_counter;       // bytes[6] contains the marker and roll counter
      }
      regular;
      struct  AlphaBeta
      {
         uint16_t alpha;                    // bytes[0] and bytes[1] 14 LSBs contain the alpha angle
         uint16_t beta;                     // bytes[2] and bytes[3] 14 LSBs contain the beta angle
         uint8_t  virtual_gain;             // bytes[4] contains the virtual gain
      }
      alpha_beta;
      struct  XYZ
      {
         uint16_t x;                        // bytes[0] and bytes[1] 14 LSBs contain the x component
         uint16_t y;                        // bytes[2] and bytes[3] 14 LSBs contain the y component
         uint16_t z;                        // bytes[4] and bytes[5] 14 LSBs contain the z component
      }
      x_y_z;
      struct  Other                         // All non regular (other) messages contain marker/opcode and CRC
      {
         uint8_t varying[message_size - 2]; // bytes[0] to bytes[5] have varying content
         uint8_t marker_opcode;             // bytes[6] contains the marker and opcode
      }
      other;
      struct  MemoryRead                    // Read memory message (two 16 bit words from two addresses)
      {
         uint16_t data0;                    // bytes[0] and bytes[1] are 16 bit word at 1st address
         uint16_t data1;                    // bytes[2] and bytes[3] are 16 bit word at 2nd address
      }
      memory_read;
      struct  EEChallenge
      {
         uint16_t unused;                   // bytes[0] and bytes[1] are unused
         uint16_t challenge_key;            // bytes[2] and bytes[3] contain the challenge key
      }
      ee_challenge;
      struct  EEStatus
      {
         uint8_t code;                      // bytes [0] contains in the 4 LSBs the code 
      }
      ee_status;
      struct  NOP                           // No operation message
      {
         uint16_t unused;                   // bytes[0] and bytes[1] are unused
         uint16_t key_echo;                 // bytes[2] and bytes[3] contain the echoed key
         uint16_t inverted_key_echo;        // bytes[4] and bytes[5] contain the binary inverted echoed key
      }
      nop;
      struct  Diagnostic                    // Diagnostic message
      {
         uint8_t diagnostic_bits[3];        // bytes[0] to bytes[2] contain the diagnostic bits
         uint8_t fsmerc_anadiagcnt;         // bytes[3] contain FSMERC [7:6] and ANADIAGCNT [5:0]
      }
      diagnostic;
      struct  OscCntrStop
      {
         uint16_t unused;                   // bytes[0] and bytes[1] are unused
         uint16_t counter_value;            // bytes[2] and bytes[3] contain CounterValue [14:0]       
      }
      osc_cntr_stop;
      struct  Ready                         // Ready message
      {
         uint8_t hw_version;                // bytes[0] contains the hardware version
         uint8_t fw_version;                // bytes[1] contains the firmware version
      }
      ready;
      struct  Error                         // Error message
      {
         ErrorCode error_code;              // bytes[0] contains the error code
      }
      error;
   };

   typedef struct AddressKey
   {
      uint16_t address;
      uint16_t key;
   };

   static constexpr uint8_t addresses          = 18;
   static constexpr uint8_t addresses_writable = 15;
   
   const AddressKey addresses_keys[addresses] = { // Writable
                                                  { 0x1000, 17485 },    // [ 0] Address::pinfilter
                                                  { 0x1018, 13134 },    // [ 1] Address::freea
                                                  { 0x1022, 64477 },    // [ 2] Address::kalpha
                                                  { 0x1024, 40905 },    // [ 3] Address::kbeta
                                                  { 0x1026, 45498 },    // [ 4] Address::orth_b1b2_freeb
                                                  { 0x1028, 24411 },    // [ 5] Address::freec_fhyst
                                                  { 0x102A, 36677 },    // [ 6] Address::mapxyz_3d_filter
                                                  { 0x102C,  4213 },    // [ 7] Address::orth_sel
                                                  { 0x102E, 48843 },    // [ 8] Address::virtualgain
                                                  { 0x1030,  6368 },    // [ 9] Address::kt
                                                  { 0x1032,  5907 },    // [10] Address::smism_sel_smism
                                                  { 0x1038,  3562 },    // [11] Address::orth
                                                  { 0x103A, 19816 },    // [12] Address::userid0
                                                  { 0x103C,  6995 },    // [13] Address::userid1
                                                  { 0x103E,  3147 },    // [14] Address::freed
                                                  // Read-Only
                                                  { 0x1012,     0 },    // [15] Address::mlxid0
                                                  { 0x1014,     0 },    // [16] Address::mlxid1
                                                  { 0x1016,     0 }  }; // [17] Address::mlxid2

   typedef union Config
   {
      uint16_t words[addresses]; // 2 byte / 16 bit EEPROM words, non-sequential (!), 36 bytes total
      struct Values // 8, 16, or 32 bit values
      {                             // EEPROM Address  Word Byte   Bits Parameter       Chapter
         uint8_t  undocumented1;    // 0x1000             0    0  [7:0] undocumented
         uint8_t  pinfilter;        //                         1  [1:0] PINFILTER       15, 16.8
                                    //                            [7:2] undocumented
         uint8_t  free0;            // 0x1018             1    0  [7:0] FREE            15, 16.9
         uint8_t  free1;            //                         1  [7:0] FREE            15, 16.9
         uint16_t kalpha;           // 0x1022             2  0-1 [15:0] KALPHA          15, 16.4
         uint16_t kbeta;            // 0x1024             3  0-1 [15:0] KBETA           15, 16.4
         uint8_t  orth_b1b2;        // 0x1026             4    0  [7:0] ORTH_B1B2       15, 16.3
         uint8_t  free2;            //                         1  [7:0] FREE            15, 16.9
         uint8_t  free3;            // 0x1028             5    0  [7:0] FREE            15, 16.9
         uint8_t  fhyst;            //                         1  [7:0] FHYST           15, 16.7
         uint8_t  mapxyz_3d_filter; // 0x102A             6    0  [2:0] MAPXYZ          15, 16.1/3
                                    //                              [3] 3D              15, 16.2
                                    //                            [5:4] FILTER          15, 16.5
                                    //                            [7:6] undocumented
         uint8_t  undocumented2;    //                         1  [7:0] undocumented
         uint8_t  undocumented3;    // 0x102C             7    0  [7:0] undocumented
         uint8_t  orth_sel;         //                         1    [0] ORTH_SEL        --, 16.3
                                    //                            [7:1] undocumented
         uint8_t  virtualgainmin;   // 0x102E             8    0  [7:0] VIRTUALGAINMIN  15, 16.6
         uint8_t  virtualgainmax;   //                         1  [7:0] VIRTUALGAINMAX  15, 16.6
         uint16_t kt;               // 0x1030             9  0-1 [15:0] KT              15, 16.4
         uint16_t smism_sel_smism;  // 0x1032            10  0-1 [14:0] SMISM           15, 16.3
                                    //                             [15] SEL_SMISM       15, 16.3
         uint8_t  orth_xy;          // 0x1038            11    0  [7:0] ORTH(_XY)       --, 16.3
         uint8_t  undocumented4;    //                         1  [7:0] undocumented
         uint32_t userid;           // 0x103A-0x103C  12-13  0-3 [31:0] USERID          15, 16.9
         uint8_t  free4;            // 0x103E            14    0  [7:0] FREE            15, 16.9
         uint8_t  undocumented5;    //                         1  [7:0] undocumented
         uint16_t mlxid[3];         // 0x1012-0x1016  15-17  0-5 [47:0] MLXID           14
      }
      values;    
   };

   SPIClass&    spi;                // Reference to SPIClass object to be used for SPI communication
   uint8_t      spi_ss_pin;         // Slave select pin used for MLX90636
   uint32_t     spi_ss_pin_high;    // Timestamp (microseconds - uses micros()) when slave select pin was set high 
   uint32_t     spi_speed;          // Speed (SPISettings(speed, ...)) for SPI communication
   MISOMessage  miso_message;       // Last received
   Config       active_config;      // Active configuration (within MLX90636 EEPROM) 
   Config       target_config;      // Target configuration (to be written to MLX90636 EEPROM)
   LibraryError last_library_error; // Holds the last error encountered by a method of this library
   uint8_t      hw_version;         // Holds the HW version from the ready MISO message received during begin()
   uint8_t      fw_version;         // Holds the FW version from the ready MISO message received during begin()

   bool     sendGet(Marker     marker,                    // Send a get messages (marker alpha, alpha_beta, x_y_z)
                    MOSIOpCode opcode,                   // with a certain trigger mode (opcode get1, get2, get3).
                    bool       reset_roll_counter,        // Allows to reset the roll counter and
                    uint16_t   data_timeout_us    );      // to set the data timeout.
   bool     validMISOMarkerOpCode(uint8_t marker_opcode); // Tests if a MISO marker & opcode is valid
   uint8_t  crc(uint8_t bytes[]);                         // Calculates the CRC of the first 7 bytes in the array
   bool     send(MOSIMessage& mosi_message);              // Sends a MOSI message (and receives a MISO message)
   bool     sendOscCntrStart();
   bool     sendOscCntrStop();
   bool     sendMemoryRead(uint8_t addr0, uint8_t addr1); // Sends a memory read MOSI message
   bool     sendEEWrite(uint8_t address, uint16_t data_word);
   bool     sendEERequest();
   bool     sendEEResponse(uint16_t challenge_key);
   uint16_t receivedOscCntrStopCounter();  
   uint16_t receivedMemoryReadData0(); // Data at addr0
   uint16_t receivedMemoryReadData1(); // Data at addr1
   uint16_t receivedEEChallengeChallengeKey();
   EECode   receivedEEStatusCode();
   bool     configRead();                   // Reads the actual configuration of the MLX90363 from its EEPROM.
   bool     eepromWrite(uint8_t  address,   // Writes a 16 bit word to the EEPROM 
                        uint16_t word    );
};

class MLX90363SyncPulse
{
   public:
   
   // Default Constructor
   // -------------------
   // Is not supported.
   MLX90363SyncPulse() = delete;

   // Constructor
   // -----------
   // uint8_t* spi_ss_pins:         Array of slave select pins of MLX90363s on SPI bus.
   // uint8_t  spi_ss_pins_number:  Number of slave select pins of MLX90363s on SPI bus.
   MLX90363SyncPulse(uint8_t* spi_ss_pins,
                     uint8_t  spi_ss_pins_number);

   void sendSyncPulse();

   private:

   uint8_t* spi_ss_pins;
   uint8_t  spi_ss_pins_number;
};

#endif
