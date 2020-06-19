/*
  xdrv_92_pid.ino - PID algorithm plugin for Sonoff-Tasmota
  Copyright (C) 2018 Colin Law and Thomas Herrmann
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * Usage:
 * In my_user_config.h or user_config_override.h uncomment the line
 *
 *   #define USE_PID
 *
 * and adjust the PID_x variables as needed.  Enable TIMEPROP similarly.
 *
 * Help with using the PID algorithm and with loop tuning can be found at
 * http://blog.clanlaw.org.uk/2018/01/09/PID-tuning-with-node-red-contrib-pid.html
 * This is directed towards using the algorithm in the node-red node node-red-contrib-pid but the algorithm here is based on
 * the code there and the tuning techique described there should work just the same.
 *
**/


#ifdef USE_PID

#include "PID.h"

#define D_CMND_PID "pid_"

#define D_CMND_PID_SETPV "pv"
#define D_CMND_PID_SETSETPOINT "sp"
#define D_CMND_PID_SETPROPBAND "pb"
#define D_CMND_PID_SETINTEGRAL_TIME "ti"
#define D_CMND_PID_SETDERIVATIVE_TIME "td"
#define D_CMND_PID_SETINITIAL_INT "initint"
#define D_CMND_PID_SETDERIV_SMOOTH_FACTOR "d_smooth"
#define D_CMND_PID_SETAUTO "auto"
#define D_CMND_PID_SETMANUAL_POWER "manual_power"
#define D_CMND_PID_SETMAX_INTERVAL "max_interval"
#define D_CMND_PID_SETUPDATE_SECS "update_secs"

enum PIDCommands { CMND_PID_SETPV, CMND_PID_SETSETPOINT, CMND_PID_SETPROPBAND, CMND_PID_SETINTEGRAL_TIME,
  CMND_PID_SETDERIVATIVE_TIME, CMND_PID_SETINITIAL_INT, CMND_PID_SETDERIV_SMOOTH_FACTOR, CMND_PID_SETAUTO,
  CMND_PID_SETMANUAL_POWER, CMND_PID_SETMAX_INTERVAL, CMND_PID_SETUPDATE_SECS };
const char kPIDCommands[] PROGMEM = D_CMND_PID_SETPV "|" D_CMND_PID_SETSETPOINT "|" D_CMND_PID_SETPROPBAND "|"
  D_CMND_PID_SETINTEGRAL_TIME "|" D_CMND_PID_SETDERIVATIVE_TIME "|" D_CMND_PID_SETINITIAL_INT "|" D_CMND_PID_SETDERIV_SMOOTH_FACTOR "|"
  D_CMND_PID_SETAUTO "|" D_CMND_PID_SETMANUAL_POWER "|" D_CMND_PID_SETMAX_INTERVAL "|" D_CMND_PID_SETUPDATE_SECS;

static PID pid;
static int update_secs = PID_UPDATE_SECS <= 0  ?  0  :  PID_UPDATE_SECS;   // how often (secs) the pid alogorithm is run
static int max_interval = PID_MAX_INTERVAL;
static unsigned long last_pv_update_secs = 0;
static boolean run_pid_now = false;     // tells PID_Every_Second to run the pid algorithm

static long pid_current_time_secs = 0;  // a counter that counts seconds since initialisation

void PID_Init()
{
  snprintf_P(log_data, sizeof(log_data), "PID Init");
  AddLog(LOG_LEVEL_INFO);
  pid.initialise( PID_SETPOINT, PID_PROPBAND, PID_INTEGRAL_TIME, PID_DERIVATIVE_TIME, PID_INITIAL_INT,
    PID_MAX_INTERVAL, PID_DERIV_SMOOTH_FACTOR, PID_AUTO, PID_MANUAL_POWER );
}

void PID_Every_Second() {
  static int sec_counter = 0;
  pid_current_time_secs++;    // increment time
  // run the pid algorithm if run_pid_now is true or if the right number of seconds has passed or if too long has
  // elapsed since last pv update. If too long has elapsed the the algorithm will deal with that.
  if (run_pid_now  ||  pid_current_time_secs - last_pv_update_secs > max_interval  ||  (update_secs != 0 && sec_counter++ % update_secs  ==  0)) {
    run_pid();
    run_pid_now = false;
  }
}

void PID_Show_Sensor() {
  // Called each time new sensor data available, data in mqtt data in same format
  // as published in tele/SENSOR
  // Update period is specified in TELE_PERIOD
  // e.g. "{"Time":"2018-03-13T16:48:05","DS18B20":{"Temperature":22.0},"TempUnit":"C"}"
  snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor: mqtt_data: %s", mqtt_data);
  AddLog(LOG_LEVEL_INFO);
  StaticJsonBuffer<400> jsonBuffer;
  // force mqtt_data to read only to stop parse from overwriting it
  JsonObject& data_json = jsonBuffer.parseObject((const char*)mqtt_data);
  if (data_json.success()) {
    const char* value = data_json["DS18B20"]["Temperature"];
    // check that something was found and it contains a number
    if (value != NULL && strlen(value) > 0 && (isdigit(value[0]) || (value[0] == '-' && isdigit(value[1])) ) ) {
      snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor: Temperature: %s", value);
      AddLog(LOG_LEVEL_INFO);
      // pass the value to the pid alogorithm to use as current pv
      last_pv_update_secs = pid_current_time_secs;
      pid.setPv(atof(value), last_pv_update_secs);
      // also trigger running the pid algorithm if we have been told to run it each pv sample
      if (update_secs == 0) {
        // this runs it at the next second
        run_pid_now = true;
      }
    } else {
      snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor - no temperature found");
      AddLog(LOG_LEVEL_INFO);
    }
  } else  {
    // parse failed
    snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor - json parse failed");
    AddLog(LOG_LEVEL_INFO);
  }
}


/* struct XDRVMAILBOX { */
/*   uint16_t      valid; */
/*   uint16_t      index; */
/*   uint16_t      data_len; */
/*   int16_t       payload; */
/*   char         *topic; */
/*   char         *data; */
/* } XdrvMailbox; */

boolean PID_Command()
{
  char command [CMDSZ];
  boolean serviced = true;
  uint8_t ua_prefix_len = strlen(D_CMND_PID); // to detect prefix of command

  snprintf_P(log_data, sizeof(log_data), "Command called: "
    "index: %d data_len: %d payload: %d topic: %s data: %s",
    XdrvMailbox.index,
    XdrvMailbox.data_len,
    XdrvMailbox.payload,
    (XdrvMailbox.payload >= 0 ? XdrvMailbox.topic : ""),
    (XdrvMailbox.data_len >= 0 ? XdrvMailbox.data : ""));
  AddLog(LOG_LEVEL_INFO);

  if (0 == strncasecmp_P(XdrvMailbox.topic, PSTR(D_CMND_PID), ua_prefix_len)) {
    // command starts with pid_
    int command_code = GetCommandCode(command, sizeof(command), XdrvMailbox.topic + ua_prefix_len, kPIDCommands);
    serviced = true;
    switch (command_code) {
      case CMND_PID_SETPV:
        snprintf_P(log_data, sizeof(log_data), "PID command setpv");
        AddLog(LOG_LEVEL_INFO);
        last_pv_update_secs = pid_current_time_secs;
        pid.setPv(atof(XdrvMailbox.data), last_pv_update_secs);
        // also trigger running the pid algorithm if we have been told to run it each pv sample
        if (update_secs == 0) {
          // this runs it at the next second
          run_pid_now = true;
        }
        break;

      case CMND_PID_SETSETPOINT:
        snprintf_P(log_data, sizeof(log_data), "PID command setsetpoint");
        AddLog(LOG_LEVEL_INFO);
        pid.setSp(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETPROPBAND:
        snprintf_P(log_data, sizeof(log_data), "PID command propband");
        AddLog(LOG_LEVEL_INFO);
        pid.setPb(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETINTEGRAL_TIME:
        snprintf_P(log_data, sizeof(log_data), "PID command Ti");
        AddLog(LOG_LEVEL_INFO);
        pid.setTi(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETDERIVATIVE_TIME:
        snprintf_P(log_data, sizeof(log_data), "PID command Td");
        AddLog(LOG_LEVEL_INFO);
        pid.setTd(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETINITIAL_INT:
        snprintf_P(log_data, sizeof(log_data), "PID command initial int");
        AddLog(LOG_LEVEL_INFO);
        pid.setInitialInt(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETDERIV_SMOOTH_FACTOR:
        snprintf_P(log_data, sizeof(log_data), "PID command deriv smooth");
        AddLog(LOG_LEVEL_INFO);
        pid.setDSmooth(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETAUTO:
        snprintf_P(log_data, sizeof(log_data), "PID command auto");
        AddLog(LOG_LEVEL_INFO);
        pid.setAuto(atoi(XdrvMailbox.data));
        break;

      case CMND_PID_SETMANUAL_POWER:
        snprintf_P(log_data, sizeof(log_data), "PID command manual power");
        AddLog(LOG_LEVEL_INFO);
        pid.setManualPower(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETMAX_INTERVAL:
      snprintf_P(log_data, sizeof(log_data), "PID command set max interval");
      AddLog(LOG_LEVEL_INFO);
      max_interval = atoi(XdrvMailbox.data);
      pid.setMaxInterval(max_interval);
      break;

      case CMND_PID_SETUPDATE_SECS:
        snprintf_P(log_data, sizeof(log_data), "PID command set update secs");
        AddLog(LOG_LEVEL_INFO);
        update_secs = atoi(XdrvMailbox.data) ;
        if (update_secs < 0) update_secs = 0;
        break;

      default:
        serviced = false;
  }

    if (serviced) {
      // set mqtt RESULT
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"%s\":\"%s\"}"), XdrvMailbox.topic, XdrvMailbox.data);
    }

  } else {
    serviced = false;
  }
  return serviced;
}

static void run_pid()
{
  double power = pid.tick(pid_current_time_secs);
  char buf[10];
  dtostrfd(power, 3, buf);
  snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"%s\":\"%s\"}"), "power", buf);
  MqttPublishPrefixTopic_P(TELE, "PID", false);

#if defined PID_SHUTTER
    // send output as a position from 0-100 to defined shutter
    int pos = power * 100;
    ShutterSetPosition(PID_SHUTTER, pos);
#endif //PID_SHUTTER

#if defined PID_USE_TIMPROP
    // send power to appropriate timeprop output
    Timeprop_Set_Power( PID_USE_TIMPROP-1, power );
#endif // PID_USE_TIMPROP
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#define XDRV_92       92

boolean Xdrv92(byte function)
//boolean XDRV_92(byte function)
{
  boolean result = false;

  switch (function) {
  case FUNC_INIT:
    PID_Init();
    break;
  case FUNC_EVERY_SECOND:
    PID_Every_Second();
    break;
  case FUNC_SHOW_SENSOR:
    // only use this if the pid loop is to use the local sensor for pv
    #if defined PID_USE_LOCAL_SENSOR
      PID_Show_Sensor();
    #endif // PID_USE_LOCAL_SENSOR
    break;
  case FUNC_COMMAND:
    result = PID_Command();
    break;
  }
  return result;
}

#endif // USE_TIMEPROP
