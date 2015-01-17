#ifndef SOUS_VIDE_H
#define SOUS_VIDE_H

// Libraries for the DS18B20 Temperature Sensor
#include "../OneWire/OneWire.h"
#include "../DallasTemperature/DallasTemperature.h"

// PID Library
#include "../PID_v1/PID_v1.h"
#include "../PID_AutoTune_v0/PID_AutoTune_v0.h"

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

const int WindowSize = 10000;  // 10 second Time Proportional Output window

// Abstract class to represent an Internal state of the machine
class State {
public:
  // must be called at least
  virtual void run();
  virtual void do_control();
};

class OffState : public State {};

class RunState : public State {};

class AutoTuneState : public RunState {
private:
  PID_ATune aTune;

public:
  AutoTuneState() : aTune(&input, &output) (){};
};

class Cooker {

friend class State;  // Internal State classes have access to the class

private:
  int relay_pin;  // The output relay that drive the physical cooker
  State* internal_state;
  double output, temperature, setpoint, Kp, Ki, Kd;;  // variables for the PID
  OneWire oneWire;
  DallasTemperature sensors;
  DeviceAddress tempSensor;
  PID myPID;

public:
  Cooker(int relay_pin = 13, int one_wire_pin = 2) :
  relay_pin(relay_pin), oneWire(one_wire_pin),
  sensors(&oneWire) {
    pinMode(relay_pin, OUTPUT);
    pinMode(one_wire_pin, INPUT);
    load_parameters();
    myPID = PID (&temperature, &output, &setpoint, Kp, Ki, Kd, DIRECT);
    myPID.SetTunings(Kp,Ki,Kd);
    myPID.SetSampleTime(1000);
    myPID.SetOutputLimits(0, WindowSize);
    set_state(new OffState());
    // Start up the DS18B20 One Wire Temperature Sensor
    sensors.begin();
    if (!sensors.getAddress(tempSensor, 0))
      Serial.println("Sensor Error");
    sensors.setResolution(tempSensor, 12);
    sensors.setWaitForConversion(false);
  }

private:
  // Update the temparature if some data is available from the sensor
  void update_temperature() {
    // Read the input:
    if (sensors.isConversionAvailable(0))
    {
      temperature = sensors.getTempC(tempSensor);
      sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
    }
  }

  void set_state(State *state);

  const double get_Kp() {return Kp;}

  void set_Kp(const double value) {Kp = value;};

  const double get_Ki() {return Ki;}

  void set_Ki(const double value) {Ki = value;};

  const double get_Kd() {return Kd;}

  void set_Kd(const double value) {Kd = value;};

  const double get_Temperature() {return setpoint;}

  void set_Temperature(const double value) {setpoint = value;};

private:
  void load_parameters();
  void save_parameters();
};

#endif  // SOUS_VIDE_H
