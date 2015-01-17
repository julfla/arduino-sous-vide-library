#include "SousVide.h"
#include "helper.h"

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void Cooker::save_parameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void Cooker::load_parameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);

   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }
}

void Cooker::set_state(State *state) {
  if (internal_state != NULL)
    delete internal_state;
  internal_state = state;
}

void OffState::run()
{
  myPID.SetMode(MANUAL);
  pinMode(relay_pin, LOW);
}

RunState::RunState() {
  myPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  SaveParameters();
  myPID.SetTunings(Kp,Ki,Kd);
}

AutoTune::AutoTune()
{
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
}

AutoTune::~AutoTune()
{
  Kp = aTune.GetKp();
  Ki = aTune.GetKi();
  Kd = aTune.GetKd();
  myPID.SetTunings(Kp,Ki,Kd);
  save_parameters();
}
