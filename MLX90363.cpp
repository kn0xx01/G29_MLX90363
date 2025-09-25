// ---------------------------------------------------------------------------------------------------- //
// Robert's Smorgasbord 2024                                                                            //
// https://robertssmorgasbord.net                                                                       //
// https://www.youtube.com/@robertssmorgasbord                                                          //
// Melexis MLX90363 Triaxis® Magnetometer & Arduino MCU – The Details (15) https://youtu.be/v6Q-92aF9bA //
// ---------------------------------------------------------------------------------------------------- //

#include "MLX90363.h"

MLX90363::MLX90363(SPIClass& spi,
                   uint8_t   spi_ss_pin,
                   uint32_t  spi_speed = spi_speed_max)
: spi(spi),
  spi_ss_pin(spi_ss_pin),
  spi_speed(spi_speed)
{    
}

bool MLX90363::begin()
{
   bool success;
   
   pinMode(spi_ss_pin, OUTPUT);
   digitalWrite(spi_ss_pin, HIGH);
   delay(1);

   sendNOP();
   delayMicroseconds(136); // t(short): 120us@5V, 135us@3.3V

   if (!sendStandby()) return false;

   if (received() != Received::nop)
   {
      last_library_error = LibraryError::opcode;
      
      return false;
   }

   delayMicroseconds(135); // t(short): 120us@5V, 135us@3.3V

   if (!sendReboot()) return false;

   if (received() != Received::standby)
   {
      last_library_error = LibraryError::opcode;
      
      return false;
   }

   delay(24); // t(reboot): 20ms@5V, 23.5ms@3.3V

   if (!sendNOP()) return false;
    
   if (received() != Received::ready)
   {
      last_library_error = LibraryError::opcode;
      
      return false;
   }

   hw_version = receivedReadyHWVersion();
   fw_version = receivedReadyFWVersion();

   if (!configRead()) return false;

   return true;
}

uint8_t MLX90363::infoFWVersion()
{
   return fw_version;
}

uint8_t MLX90363::infoHWVersion()
{
   return hw_version;
}

uint64_t MLX90363::infoMLXID()
{
   uint64_t mlxid = 0;

   for (int8_t word = 2; word >= 0; word--)
   {
      mlxid = mlxid << 16;
      mlxid = mlxid | active_config.values.mlxid[word];
   }

   return mlxid;
}

// Configuration Parameters
// ========================
// Active in struct active_config.values
// Update in struct target_config.values

// uint_8 pinfilter
// ----------------

// 16.8 PINFILTER
// Bits [1..0], enum

MLX90363::PinFilter MLX90363::activePinFilter() 
{
   return (PinFilter)(active_config.values.pinfilter & bitmask_pinfilter);
}

void MLX90363::updatePinFilter(MLX90363::PinFilter to) 
{
   target_config.values.pinfilter &= ~bitmask_pinfilter;
   target_config.values.pinfilter |= (uint8_t)to;
}

// uint_8 free0 ... uint_8 free4
// -----------------------------

// 16.9 FREE
// Bits [7:0] x 5

MLX90363::Free MLX90363::activeFree()
{
   Free free;

   free.bytes[0] = active_config.values.free0;
   free.bytes[1] = active_config.values.free1;
   free.bytes[2] = active_config.values.free2;
   free.bytes[3] = active_config.values.free3;
   free.bytes[4] = active_config.values.free4;
    
   return free;
}

void MLX90363::updateFree(MLX90363::Free& to)
{
   Free free;

   target_config.values.free0 = free.bytes[0];
   target_config.values.free1 = free.bytes[1];
   target_config.values.free2 = free.bytes[2];
   target_config.values.free3 = free.bytes[3];
   target_config.values.free4 = free.bytes[4];
}

// uint16_t kalpha
// ---------------

// 16.4 KALPHA
// Bits [15:0], float 0..2, integer 0..65535 (2^16 - 1)

uint16_t MLX90363::activeKAlphaRaw()
{
   return active_config.values.kalpha; 
}

void MLX90363::updateKAlphaRaw(uint16_t to)
{
   target_config.values.kalpha = to; 
}

float MLX90363::activeKAlpha()
{
   return (float)activeKAlphaRaw() * 2.0 / 65535.0;
}

void MLX90363::updateKAlpha(float to)
{
   if (to < 0.0 || to > 2.0) return;
   
   updateKAlphaRaw(round(to / 2.0 * 65535.0)); 
}

// uint16_t kbeta
// --------------

// 16.4 KBETA
// Bits [15:0], float 0..2, integer 0..65535 (2^16 - 1)

uint16_t MLX90363::activeKBetaRaw()
{
   return active_config.values.kbeta; 
}

void MLX90363::updateKBetaRaw(uint16_t to)
{
   target_config.values.kbeta = to; 
}

float MLX90363::activeKBeta()
{
   return (float)activeKBetaRaw() * 2.0 / 65535.0;
}

void MLX90363::updateKBeta(float to)
{
   if (to < 0.0 || to > 2.0) return;
   
   updateKBetaRaw(round(to / 2.0 * 65535.0)); 
}

// uint8_t orth_b1b2
// -----------------

// 16.3 ORTH_B1B2
// Bits [7:0], float 0..2, integer 0..255 (2^8 - 1)

uint8_t MLX90363::activeOrthB1B2Raw()
{
   return active_config.values.orth_b1b2; 
}

void MLX90363::updateOrthB1B2Raw(uint8_t to)
{
   target_config.values.orth_b1b2 = to; 
}

float MLX90363::activeOrthB1B2()
{
   return (float)activeOrthB1B2Raw() * 2.0 / 255.0;
}

void MLX90363::updateOrthB1B2(float to)
{
   if (to < 0.0 || to > 2.0) return;
   
   updateOrthB1B2Raw(round(to / 2.0 * 255.0)); 
}

// uint8_t fhyst
// -------------

// 16.7 FHYST
// Bits [7:0], range 0..11.22, 1 LSB = 0.044°

uint8_t MLX90363::activeFHystRaw()
{
   return active_config.values.fhyst;
}

void MLX90363::updateFHystRaw(uint8_t to)
{
   target_config.values.fhyst = to;
}

float MLX90363::activeFHyst()
{
   return (float)activeFHystRaw() * 0.044;  
}

void MLX90363::updateFHyst(float to)
{
   if (to < 0.0 || to > 11.22) return;
   
   updateFHystRaw(round(to / 0.044));
}

// uint_8 mapxyz_3d_filter
// -----------------------

// 16.1 MAPXYZ
// Bits [2:0], enum

MLX90363::MapXYZ MLX90363::activeMapXYZ()
{
   return (MapXYZ)(active_config.values.mapxyz_3d_filter & bitmask_mapxyz);
}

void MLX90363::updateMapXYZ(MLX90363::MapXYZ to)
{
   target_config.values.mapxyz_3d_filter &= ~bitmask_mapxyz;
   target_config.values.mapxyz_3d_filter |= (uint8_t)to; 
}

// 16.2 3D
// Bit [3], boolean

bool MLX90363::active3D()
{
   return (active_config.values.mapxyz_3d_filter & bitmask_3d ? true : false);
}

void MLX90363::update3D(bool to)
{
   target_config.values.mapxyz_3d_filter &= ~bitmask_3d;
   target_config.values.mapxyz_3d_filter |= (to ? bitmask_3d : (uint8_t)0);
}

// 16.5 FILTER
// Bits [5:4], enum

MLX90363::Filter MLX90363::activeFilter()
{
   return (Filter)(active_config.values.mapxyz_3d_filter & bitmask_filter);
}

void MLX90363::updateFilter(MLX90363::Filter to)
{
   target_config.values.mapxyz_3d_filter &= ~bitmask_filter;
   target_config.values.mapxyz_3d_filter |= (uint8_t)to;
}

// uint8_t orth_sel
// ----------------

// 16.3 ORTH_SEL
// Bit [0], enum

MLX90363::OrthSel MLX90363::activeOrthSel()       
{
   return (active_config.values.orth_sel & bitmask_orth_sel ? 
           OrthSel::b1b2                                    : 
           OrthSel::xy                                       );
}

void MLX90363::updateOrthSel(OrthSel to)       
{
   target_config.values.orth_sel &= ~bitmask_orth_sel;
   target_config.values.orth_sel |= ((uint8_t)to ? bitmask_orth_sel : (uint8_t)0);
}

// uint8_t virtualgainmin
// ----------------------

// 16.6 VIRTUALGAINMIN
// Bits [7:0], integer 0..255

uint8_t MLX90363::activeVirtualGainMin()
{
   return active_config.values.virtualgainmin;
}

void MLX90363::updateVirtualGainMin(uint8_t to)
{
   target_config.values.virtualgainmin = to;
}

// uint8_t virtualgainmax
// ----------------------

// 16.6 VIRTUALGAINMAX
// Bits [7:0], integer 0..255

uint8_t MLX90363::activeVirtualGainMax()
{
   return active_config.values.virtualgainmax;
}

void MLX90363::updateVirtualGainMax(uint8_t to)
{
   target_config.values.virtualgainmax = to;
}

// uint16_t kt
// -----------

// 16. KT
// Bits [15:0], float 0..2, integer 0..65535 (2^16 - 1)

uint16_t MLX90363::activeKTRaw()
{
   return active_config.values.kt; 
}

void MLX90363::updateKTRaw(uint16_t to)
{
   target_config.values.kt = to; 
}

float MLX90363::activeKT()
{
   return (float)activeKTRaw() * 2.0 / 65535.0;
}

void MLX90363::updateKT(float to)
{
   if (to < 0.0 || to > 2.0) return;
   
   updateKTRaw(round(to / 2.0 * 65535.0)); 
}

// uint16_t smism_sel_smism
// ------------------------

// 16.3 SMISM
// Bits [14..0], float 0..2, integer 0..32767 (2^15 - 1)

uint16_t MLX90363::activeSMismRaw()
{
   return active_config.values.smism_sel_smism & bitmask_smism; 
}

void MLX90363::updateSMismRaw(uint16_t to)
{
   if (to > 32767) return;

   target_config.values.smism_sel_smism &= ~bitmask_smism;
   target_config.values.smism_sel_smism |= to;
}

float MLX90363::activeSMism()
{
   return (float)activeSMismRaw() / 32768.0; // 2^15 
}

void MLX90363::updateSMism(float to)
{  
   if (to < 0 || to > 1) return;
    
   updateSMismRaw(round(to * 32768.0) > 32767 ? 32767 : round(to * 32768.0));
}

// 16.3 SEL_SMISM
// Bit [15], enumeration (boolean)

MLX90363::SelSMism MLX90363::activeSelSMism()
{
   return (active_config.values.smism_sel_smism & bitmask_sel_smism ? 
           SelSMism::multiply_b2                                    :   // sel_smism = 1
           SelSMism::multiply_b1                                     ); // sel_smism = 0
}

void MLX90363::updateSelSMism(MLX90363::SelSMism to)
{
   target_config.values.smism_sel_smism &= ~bitmask_sel_smism;
   target_config.values.smism_sel_smism |= ((uint8_t)to ? bitmask_sel_smism : (uint16_t)0);
}

// uint8_t orth
// ------------

// 16.3 ORTH(_XY)
// Bits [7:0], integer 0..255 (2^8 - 1)

uint8_t MLX90363::activeOrthXY()
{
   return active_config.values.orth_xy;
}

// uint32_t userid
// ---------------

// 16.9 USERID
// Bits [31:0]

uint32_t MLX90363::activeUserID()
{
   return active_config.values.userid;
}

void MLX90363::updateUserID(uint32_t to)
{
   target_config.values.userid = to;
}

// Send
// ====

bool MLX90363::sendGet1Alpha(bool     reset_roll_counter = false,
                             uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::alpha, MOSIOpCode::get1, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet2Alpha(bool     reset_roll_counter = false,
                             uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::alpha, MOSIOpCode::get2, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet3Alpha(bool     reset_roll_counter = false,
                             uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::alpha, MOSIOpCode::get3, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet1AlphaBeta(bool     reset_roll_counter = false,
                                 uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::alpha_beta, MOSIOpCode::get1, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet2AlphaBeta(bool     reset_roll_counter = false,
                                 uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::alpha_beta, MOSIOpCode::get2, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet3AlphaBeta(bool     reset_roll_counter = false,
                                 uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::alpha_beta, MOSIOpCode::get3, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet1XYZ(bool     reset_roll_counter = false,
                           uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::x_y_z, MOSIOpCode::get1, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet2XYZ(bool     reset_roll_counter = false,
                           uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::x_y_z, MOSIOpCode::get2, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendGet3XYZ(bool     reset_roll_counter = false,
                           uint16_t data_timeout_us    = 0xFFFF )
{
   return sendGet(Marker::x_y_z, MOSIOpCode::get3, reset_roll_counter, data_timeout_us);
}

bool MLX90363::sendMemoryRead(uint8_t addr0, uint8_t addr1)
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::memory_read );

   mosi_message.memory_read.addr0 = addresses_keys[addr0].address;
   mosi_message.memory_read.addr1 = addresses_keys[addr1].address;

   return send(mosi_message);
}

bool MLX90363::sendEEWrite(uint8_t address, uint16_t data_word)
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::ee_write );

   mosi_message.ee_write.address   = addresses_keys[address].address & 0b0000000000111111;
   mosi_message.ee_write.key       = addresses_keys[address].key;
   mosi_message.ee_write.data_word = data_word;

   return send(mosi_message);
}

bool MLX90363::sendEERequest()
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::ee_request );

   return send(mosi_message);
}

bool MLX90363::sendEEResponse(uint16_t challenge_key)
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::ee_response );

   mosi_message.ee_response.key_echo          =   challenge_key ^ 0x1234;
   mosi_message.ee_response.inverted_key_echo = ~(challenge_key ^ 0x1234);

   return send(mosi_message);
}

bool MLX90363::sendNOP(uint16_t key = 0)
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::nop );

   mosi_message.nop.key = key;

   return send(mosi_message);
}

bool MLX90363::sendReboot()
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::reboot );

   return send(mosi_message);
}

bool MLX90363::sendStandby()
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::standby );

   return send(mosi_message);
}

bool MLX90363::sendDiagnostic()
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::diagnostic );

   return send(mosi_message);
}

bool MLX90363::sendOscCntrStart()
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::osc_cntr_start );

   return send(mosi_message);
}

bool MLX90363::sendOscCntrStop()
{
   MOSIMessage mosi_message( Marker::other, MOSIOpCode::osc_cntr_stop );

   return send(mosi_message);
}

MLX90363::LibraryError MLX90363::libraryError()
{
   return last_library_error;
}

// Received
// --------

MLX90363::Received MLX90363::received()
{
   if ((Marker)(miso_message.all.marker & bitmask_marker) == Marker::other)
   {
      return (Received)miso_message.other.marker_opcode;
   }
   else
   {
      return (Received)(miso_message.regular.marker_roll_counter & bitmask_marker);
   }
}

MLX90363::Diagnostic MLX90363::receivedGetDiagnostic()
{
   return (Diagnostic)(miso_message.regular.diagnostic >> 6);
}

uint8_t MLX90363::receivedGetRollCounter()
{
   return (miso_message.regular.marker_roll_counter & bitmask_roll_counter);
}

uint8_t MLX90363::receivedGetVirtualGain()
{
   return miso_message.alpha_beta.virtual_gain;
}

uint16_t MLX90363::receivedGetAlphaRaw()
{
   return (miso_message.alpha_beta.alpha & 0b0011111111111111);
}

float MLX90363::receivedGetAlphaDeg()
{
   return (float)receivedGetAlphaRaw() * 360.0 / 16384.0; // ... * 360° / 2^14
}

uint16_t MLX90363::receivedGetBetaRaw()
{
   return (miso_message.alpha_beta.beta & 0b0011111111111111);
}

float MLX90363::receivedGetBetaDeg()
{
   return (float)receivedGetBetaRaw() * 360.0 / 16384.0; // ... * 360° / 2^14
}

int16_t MLX90363::receivedGetX()
{
   return ((int16_t)(miso_message.x_y_z.x << 2))/4;
}

int16_t MLX90363::receivedGetY()
{
   return ((int16_t)(miso_message.x_y_z.y << 2))/4;
}

int16_t MLX90363::receivedGetZ()
{
   return ((int16_t)(miso_message.x_y_z.z << 2))/4;
}

uint16_t MLX90363::receivedMemoryReadData0()
{
   return miso_message.memory_read.data0;
}

uint16_t MLX90363::receivedMemoryReadData1()
{
   return miso_message.memory_read.data1;
}

uint16_t MLX90363::receivedEEChallengeChallengeKey()
{
   return miso_message.ee_challenge.challenge_key;
}

MLX90363::EECode MLX90363::receivedEEStatusCode()
{
   return (EECode)(miso_message.ee_status.code & bitmask_code);
}

uint16_t MLX90363::receivedNOPKeyEcho()
{
   return miso_message.nop.key_echo;
}

uint16_t MLX90363::receivedNOPInvertedKeyEcho()
{
   return miso_message.nop.inverted_key_echo;
}

MLX90363::DiagItems MLX90363::receivedDiagnosticDiagItems()
{
   return (*(uint32_t*)(miso_message.diagnostic.diagnostic_bits)) & 0x001FFFFF;
}

bool MLX90363::receivedDiagnosticDiagItems(MLX90363::DiagItems items)
{
   return ((*(uint32_t*)(miso_message.diagnostic.diagnostic_bits)) & items) != 0x00000000;
}

MLX90363::FailSafeCause MLX90363::receivedDiagnosticFailSafeCause()
{
   return (FailSafeCause)(miso_message.diagnostic.fsmerc_anadiagcnt & bitmask_fsmerc);
}

MLX90363::FailSafeError MLX90363::receivedDiagnosticFailSafeError()
{
   if (receivedDiagnosticFailSafeCause() != FailSafeCause::error) return FailSafeError::none;

   return (FailSafeError)(miso_message.diagnostic.fsmerc_anadiagcnt & bitmask_anadiagcnt);
}

uint16_t MLX90363::receivedOscCntrStopCounter()
{
   return miso_message.osc_cntr_stop.counter_value & bitmask_counter_value;
}

int8_t MLX90363::receivedDiagnosticLoopCounter()
{
   if (receivedDiagnosticFailSafeCause() == FailSafeCause::error) return -1;

   return (miso_message.diagnostic.fsmerc_anadiagcnt & bitmask_anadiagcnt);
}

uint8_t MLX90363::receivedReadyHWVersion()
{
   return miso_message.ready.hw_version;
}

uint8_t MLX90363::receivedReadyFWVersion()
{
   return miso_message.ready.fw_version;
}

MLX90363::ErrorCode MLX90363::receivedErrorErrorCode()
{
   return (ErrorCode)(miso_message.error.error_code);
}

bool MLX90363::sendGet(Marker     marker,
                       MOSIOpCode opcode,
                       bool       reset_roll_counter,
                       uint16_t   data_timeout_us    )
{
   MOSIMessage mosi_message( marker, opcode );

   mosi_message.get.reset_roll_counter = (reset_roll_counter ? 0b00000001 : 0b00000000);
   mosi_message.get.data_timeout_us    = data_timeout_us;     

   return send(mosi_message);
}

bool MLX90363::configRead()
{
   if (!sendMemoryRead(0, 1)) return false;

   for (uint8_t address = 2; address < addresses; address += 2)
   {
      delayMicroseconds(139); // t(short): 120us@5V, 139us@3V3
      
      if (!sendMemoryRead(address, address + 1)) return false;

      if (received() != Received::memory_read)
      {
         last_library_error = LibraryError::opcode;
      
         return false;
      }

      active_config.words[address - 2] = receivedMemoryReadData0();
      active_config.words[address - 1] = receivedMemoryReadData1();
   }

   delayMicroseconds(139); // t(short): 120us@5V, 139us@3V3

   if (!sendNOP()) return false;

   if (received() != Received::memory_read)
   {
      last_library_error = LibraryError::opcode;
      
      return false;
   }

   active_config.words[18 - 2] = receivedMemoryReadData0();
   active_config.words[18 - 1] = receivedMemoryReadData1();
   
   target_config.values = active_config.values;

   delayMicroseconds(139); // t(short): 120us@5V, 139us@3V3

   return true;
}

bool MLX90363::commitConfig()
{
   bool reboot_required = false;
   
   for (uint8_t address = 0; address < addresses_writable; address++)
   {
      if (active_config.words[address] != target_config.words[address])
      {
         reboot_required = true;
         
         if (!eepromWrite(address, target_config.words[address])) return false;
      }
   }

   if (!reboot_required) return true;

   delayMicroseconds(135); // t(short): 120us@5V, 135us@3.3V

   if (!sendReboot()) return false;

   delay(24); // t(reboot): 20ms@5V, 23.5ms@3.3V

   if (!configRead()) return false;

   return true;
}

float MLX90363::infoOscFreqMHz(uint16_t sample_time_us = 16383)
{
   uint32_t timestamp;
   uint32_t t_osc_counter;
   
   if (sample_time_us < 500 || sample_time_us > 16383) return NAN;
   
   if (!sendOscCntrStart()) return NAN;

   timestamp = spi_ss_pin_high; // Timestamp of oscillator counter start SPI transaction end

   delayMicroseconds(sample_time_us); // Works only accurately in the range 3 microseconds and up to 16383

   if (!sendOscCntrStop()) return NAN;

   t_osc_counter = spi_ss_pin_high - timestamp; // t(OscCounter): min. 500us, typ. 1000us, max 30000us

   delayMicroseconds(139); // t(short): 120us@5V, 139us@3V3

   if (!sendNOP()) return NAN;
   
   if (received() != Received::osc_cntr_stop)
   {
      last_library_error = LibraryError::opcode;

      return NAN;
   }
   
   // Ck = 19 [MHz] * (CounterValue - 40) [lsb] / tOscCounter [μs]

   return 19.0 * (float)(receivedOscCntrStopCounter() - 40) / (float)t_osc_counter;
}

bool MLX90363::eepromWrite(uint8_t  address, 
                           uint16_t word    )
{
   if (!sendEEWrite(address, word)) return false;

   delayMicroseconds(139); // t(short): 120us@5V, 139us@3V3

   if (!sendEERequest()) return false;

   if (received() == Received::ee_status)
   {
      switch (receivedEEStatusCode())
      {
         case EECode::key_invalid: last_library_error = LibraryError::ee_key;     break;
         case EECode::odd_address: last_library_error = LibraryError::ee_address; break;
      }

      return false;
   }
      
   if (received() != Received::ee_challenge)
   {
      last_library_error = LibraryError::opcode;

      return false;
   }

   delayMicroseconds(139); // t(short): 120us@5V, 139us@3V3
   
   if (!sendEEResponse(receivedEEChallengeChallengeKey())) return false;

   if (received() != Received::ee_writing)
   {
      last_library_error = LibraryError::opcode;

      return false;
   }

   delay(37); // t(eewrite): 32ms@5V, 37ms@3V3
   
   if (!sendNOP()) return false;
   
   if (received() != Received::ee_status)
   {
      last_library_error = LibraryError::opcode;

      return false;
   }

   if (receivedEEStatusCode() != EECode::success)
   {
      switch (receivedEEStatusCode())
      {
         case EECode::erase_write_fail: last_library_error = LibraryError::ee_erase_write; break;
         case EECode::crc_fail:         last_library_error = LibraryError::ee_crc;         break;
         case EECode::challenge_fail:   last_library_error = LibraryError::ee_challenge;   break;
      }
      
      return false;
   }

   return true;
}

uint8_t MLX90363::crc(uint8_t message_bytes[])
{
   constexpr uint8_t crc_polynominal[256] = { 0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD,
                                              0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A, 
                                              0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
                                              0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
                                              0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE,
                                              0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
                                              0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10,
                                              0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
                                              0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
                                              0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
                                              0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85,
                                              0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
                                              0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58,
                                              0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
                                              0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
                                              0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
                                              0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E,
                                              0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
                                              0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80,
                                              0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
                                              0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
                                              0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
                                              0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3,
                                              0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
                                              0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8,
                                              0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
                                              0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
                                              0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
                                              0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB,
                                              0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
                                              0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15,
                                              0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42 };

   uint8_t crc_value;
   uint8_t i;

   crc_value = 0xFF;

   for (i = 0; i < message_size - 1; i++)
   {
      crc_value = crc_polynominal[crc_value ^ message_bytes[i]];
   }

   crc_value = ~crc_value;

   return crc_value;
}

bool MLX90363::validMISOMarkerOpCode(uint8_t marker_opcode)
{
   constexpr uint8_t other_opcodes[] = { (uint8_t)Received::memory_read,
                                         (uint8_t)Received::ee_challenge,
                                         (uint8_t)Received::ee_status,
                                         (uint8_t)Received::nop,
                                         (uint8_t)Received::diagnostic,
                                         (uint8_t)Received::osc_cntr_start,
                                         (uint8_t)Received::osc_cntr_stop,
                                         (uint8_t)Received::ee_writing,
                                         (uint8_t)Received::ready,
                                         (uint8_t)Received::get3ready,
                                         (uint8_t)Received::standby,
                                         (uint8_t)Received::error,
                                         (uint8_t)Received::ntt             };

   if ((Marker)(marker_opcode & bitmask_marker) == Marker::other)
   {
      for (uint8_t opcode = 0; opcode < sizeof(other_opcodes); opcode++)
      {
         if (marker_opcode == other_opcodes[opcode])
         {
            return true;                                       
         }
      }
   }
   else // Messages with marker alpha, alpha-beta and x-y-z have no opcode  
   {
      return true;
   }

   return false;
}

bool MLX90363::send(MOSIMessage& mosi_message)
{
   uint8_t i;

   mosi_message.all.crc = crc(mosi_message.bytes);

   spi.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
   digitalWrite(spi_ss_pin, LOW);

   for (i = 0; i < message_size; i++) 
   {
      miso_message.bytes[i] = spi.transfer(mosi_message.bytes[i]);
   }
   
   digitalWrite(spi_ss_pin, HIGH);
   spi.endTransaction();

   spi_ss_pin_high = micros(); // Only needed for oscillator counter measurements

   if (miso_message.all.crc != crc(miso_message.bytes))
   {
      last_library_error = LibraryError::crc;

      return false;
   }

   if (!validMISOMarkerOpCode(miso_message.other.marker_opcode))
   {
      last_library_error = LibraryError::opcode;

      return false;
   }

   return true;
}

MLX90363SyncPulse::MLX90363SyncPulse(uint8_t spi_ss_pins[],
                                     uint8_t spi_ss_pins_number)
: spi_ss_pins(spi_ss_pins),
  spi_ss_pins_number(spi_ss_pins_number)
{    
}

void MLX90363SyncPulse::sendSyncPulse()
{
   // Just for demonstration purposes!
   // You would use PORT[B|C|D] = ... to set all ss pins at once.
   
   uint8_t spi_ss_pin;

   for (spi_ss_pin = 0; spi_ss_pin < spi_ss_pins_number; spi_ss_pin++)
   {
      digitalWrite(spi_ss_pins[spi_ss_pin], LOW);
   }

   delayMicroseconds(3); // tSyncPulse  EE_PINFILTER    Min      Max
                         //                        1  520ns  10000ns
                         //                        2  610ns  10000ns
                         //                        3  820ns  10000ns

   for (spi_ss_pin = 0; spi_ss_pin < spi_ss_pins_number; spi_ss_pin++)
   {
      digitalWrite(spi_ss_pins[spi_ss_pin], HIGH);
   }
}
