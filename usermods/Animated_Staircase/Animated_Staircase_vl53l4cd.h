/*
 * Usermod for detecting people entering/leaving a staircase and switching the
 * staircase on/off.
 *
 * Edit the Animated_Staircase_config.h file to compile this usermod for your
 * specific configuration.
 *
 * See the accompanying README.md file for more info.
 */
#pragma once
#include "wled.h"
#include "Wire.h"
#include "vl53l4cd_class.h"

class Animated_Staircase_vl53l4cd : public Usermod
{
private:
    /* configuration (available in API and stored in flash) */
    bool enabled = false;                 // Enable this usermod
    unsigned long segment_delay_ms = 150; // Time between switching each segment
    unsigned long on_time_ms = 30000;     // The time for the light to stay on
    int8_t topXShutPin = -1;              // disabled
    int8_t bottomXShutPin = -1;           // disabled
    unsigned int topMaxDist = 50;         // default maximum measured distance in cm, top
    unsigned int bottomMaxDist = 50;      // default maximum measured distance in cm, bottom
    bool togglePower = false;             // toggle power on/off with staircase on/off

#define DEV_I2C Wire

    // VL53L4CD sensor
    VL53L4CD topVL53L4CDSensor = VL53L4CD(&DEV_I2C, topXShutPin);

    /* runtime variables */
    bool initDone = false;

    // Time between checking of the sensors
    const unsigned int scanDelay = 100;

    // Lights on or off.
    // Flipping this will start a transition.
    bool on = false;

    // Swipe direction for current transition
#define SWIPE_UP true
#define SWIPE_DOWN false
    bool swipe = SWIPE_UP;

    // Indicates which Sensor was seen last (to determine
    // the direction when swiping off)
#define LOWER false
#define UPPER true
    bool lastSensor = LOWER;

    // Time of the last transition action
    unsigned long lastTime = 0;

    // Time of the last sensor check
    unsigned long lastScanTime = 0;

    // Last time the lights were switched on or off
    unsigned long lastSwitchTime = 0;

    // segment id between onIndex and offIndex are on.
    // controll the swipe by setting/moving these indices around.
    // onIndex must be less than or equal to offIndex
    byte onIndex = 0;
    byte offIndex = 0;

    // The maximum number of configured segments.
    // Dynamically updated based on user configuration.
    byte maxSegmentId = 1;
    byte minSegmentId = 0;

    // These values are used by the API to read the
    // last sensor state, or trigger a sensor
    // through the API
    bool topSensorRead = false;
    bool topSensorWrite = false;
    bool bottomSensorRead = false;
    bool bottomSensorWrite = false;
    bool topSensorState = false;
    bool bottomSensorState = false;

    // strings to reduce flash memory usage (used more than twice)
    static const char _name[];
    static const char _enabled[];
    static const char _segmentDelay[];
    static const char _onTime[];
    static const char _togglePower[];
    static const char _topXShutPin[];
    static const char _bottomXShutPin[];

    void publishMqtt(bool bottom, const char *state)
    {
#ifndef WLED_DISABLE_MQTT
        // Check if MQTT Connected, otherwise it will crash the 8266
        if (WLED_MQTT_CONNECTED)
        {
            char subuf[64];
            sprintf_P(subuf, PSTR("%s/motion/%d"), mqttDeviceTopic, (int)bottom);
            mqtt->publish(subuf, 0, false, state);
        }
#endif
    }

    void updateSegments()
    {
        for (int i = minSegmentId; i < maxSegmentId; i++)
        {
            Segment &seg = strip.getSegment(i);
            if (!seg.isActive())
                continue; // skip gaps
            if (i >= onIndex && i < offIndex)
            {
                seg.setOption(SEG_OPTION_ON, true);
                // We may need to copy mode and colors from segment 0 to make sure
                // changes are propagated even when the config is changed during a wipe
                // seg.setMode(mainsegment.mode);
                // seg.setColor(0, mainsegment.colors[0]);
            }
            else
            {
                seg.setOption(SEG_OPTION_ON, false);
            }
            // Always mark segments as "transitional", we are animating the staircase
            // seg.setOption(SEG_OPTION_TRANSITIONAL, true); // not needed anymore as setOption() does it
        }
        strip.trigger();     // force strip refresh
        stateChanged = true; // inform external devices/UI of change
        colorUpdated(CALL_MODE_DIRECT_CHANGE);
    }

    bool checkSensors()
    {
        bool sensorChanged = false;

        if ((millis() - lastScanTime) > scanDelay)
        {
            lastScanTime = millis();

            uint8_t NewDataReady = 0;
            VL53L4CD_Result_t results;
            uint8_t status;

            // Check if new data is ready
            status = topVL53L4CDSensor.VL53L4CD_CheckForDataReady(&NewDataReady);
            if ((!status) && (NewDataReady != 0))
            {
                // (Mandatory) Clear HW interrupt to restart measurements
                topVL53L4CDSensor.VL53L4CD_ClearInterrupt();

                // Read measured distance. RangeStatus = 0 means valid data
                topVL53L4CDSensor.VL53L4CD_GetResult(&results);

                if (results.range_status == 0)
                {
                    topSensorRead = topSensorWrite || (results.distance_mm < topMaxDist * 10); // mm to cm
                }
                else
                {
                    topSensorRead = false;
                }
            }

            if (bottomSensorRead != bottomSensorState)
            {
                bottomSensorState = bottomSensorRead; // change previous state
                sensorChanged = true;
                publishMqtt(true, bottomSensorState ? "on" : "off");
                DEBUG_PRINTLN(F("Bottom sensor changed."));
            }

            if (topSensorRead != topSensorState)
            {
                topSensorState = topSensorRead; // change previous state
                sensorChanged = true;
                publishMqtt(false, topSensorState ? "on" : "off");
                DEBUG_PRINTLN(F("Top sensor changed."));
            }

            // Values read, reset the flags for next API call
            topSensorWrite = false;
            bottomSensorWrite = false;

            if (topSensorRead != bottomSensorRead)
            {
                lastSwitchTime = millis();

                if (on)
                {
                    lastSensor = topSensorRead;
                }
                else
                {
                    if (togglePower && onIndex == offIndex && offMode)
                        toggleOnOff(); // toggle power on if off
                    // If the bottom sensor triggered, we need to swipe up, ON
                    swipe = bottomSensorRead;

                    DEBUG_PRINT(F("ON -> Swipe "));
                    DEBUG_PRINTLN(swipe ? F("up.") : F("down."));

                    if (onIndex == offIndex)
                    {
                        // Position the indices for a correct on-swipe
                        if (swipe == SWIPE_UP)
                        {
                            onIndex = minSegmentId;
                        }
                        else
                        {
                            onIndex = maxSegmentId;
                        }
                        offIndex = onIndex;
                    }
                    on = true;
                }
            }
        }
        return sensorChanged;
    }

    void autoPowerOff()
    {
        if ((millis() - lastSwitchTime) > on_time_ms)
        {
            // if sensors are still on, do nothing
            if (bottomSensorState || topSensorState)
                return;

            // Swipe OFF in the direction of the last sensor detection
            swipe = lastSensor;
            on = false;

            DEBUG_PRINT(F("OFF -> Swipe "));
            DEBUG_PRINTLN(swipe ? F("up.") : F("down."));
        }
    }

    void updateSwipe()
    {
        if ((millis() - lastTime) > segment_delay_ms)
        {
            lastTime = millis();

            byte oldOn = onIndex;
            byte oldOff = offIndex;
            if (on)
            {
                // Turn on all segments
                onIndex = MAX(minSegmentId, onIndex - 1);
                offIndex = MIN(maxSegmentId, offIndex + 1);
            }
            else
            {
                if (swipe == SWIPE_UP)
                {
                    onIndex = MIN(offIndex, onIndex + 1);
                }
                else
                {
                    offIndex = MAX(onIndex, offIndex - 1);
                }
            }
            if (oldOn != onIndex || oldOff != offIndex)
            {
                updateSegments(); // reduce the number of updates to necessary ones
                if (togglePower && onIndex == offIndex && !offMode && !on)
                    toggleOnOff(); // toggle power off for all segments off
            }
        }
    }

    // send sensor values to JSON API
    void writeSensorsToJson(JsonObject &staircase)
    {
        staircase[F("top-sensor")] = topSensorRead;
        staircase[F("bottom-sensor")] = bottomSensorRead;
    }

    // allow overrides from JSON API
    void readSensorsFromJson(JsonObject &staircase)
    {
        bottomSensorWrite = bottomSensorState || (staircase[F("bottom-sensor")].as<bool>());
        topSensorWrite = topSensorState || (staircase[F("top-sensor")].as<bool>());
    }

    void enable(bool enable)
    {
        if (enable)
        {
            DEBUG_PRINTLN(F("Animated Staircase enabled."));
            DEBUG_PRINT(F("Delay between steps: "));
            DEBUG_PRINT(segment_delay_ms);
            DEBUG_PRINT(F(" milliseconds.\nStairs switch off after: "));
            DEBUG_PRINT(on_time_ms / 1000);
            DEBUG_PRINTLN(F(" seconds."));

            onIndex = minSegmentId = strip.getMainSegmentId(); // it may not be the best idea to start with main segment as it may not be the first one
            offIndex = maxSegmentId = strip.getLastActiveSegmentId() + 1;

            // shorten the strip transition time to be equal or shorter than segment delay
            transitionDelay = segment_delay_ms;
            strip.setTransition(segment_delay_ms);
            strip.trigger();
        }
        else
        {
            if (togglePower && !on && offMode)
                toggleOnOff(); // toggle power on if off
            // Restore segment options
            for (int i = 0; i <= strip.getLastActiveSegmentId(); i++)
            {
                Segment &seg = strip.getSegment(i);
                if (!seg.isActive())
                    continue; // skip vector gaps
                seg.setOption(SEG_OPTION_ON, true);
            }
            strip.trigger();     // force strip update
            stateChanged = true; // inform external devices/UI of change
            colorUpdated(CALL_MODE_DIRECT_CHANGE);
            DEBUG_PRINTLN(F("Animated Staircase disabled."));
        }
        enabled = enable;
    }

public:
    void setup()
    {
        // standardize invalid pin numbers to -1
        if (topXShutPin < 0)
            topXShutPin = -1;
        if (bottomXShutPin < 0)
            bottomXShutPin = -1;

        DEBUG_PRINTF("[%s] VL53L4CD bus init\n", _name);

        if (i2c_sda < 0 || i2c_scl < 0)
        {
            enabled = false;
            DEBUG_PRINTF("[%s] I2C bus not initialised!\n", _name);
            return;
        }

        PinManagerPinType pins[6];
        pins[0] = {i2c_sda, true};
        pins[1] = {i2c_scl, true};
        pins[2] = {spi_sclk, true};
        pins[3] = {spi_mosi, true};
        pins[4] = {topXShutPin, true};
        pins[5] = {bottomXShutPin, true};

        if (!pinManager.allocateMultiplePins(pins, 2, PinOwner::UM_AnimatedStaircase))
        {
            topXShutPin = -1;
            bottomXShutPin = -1;
            enabled = false;
        }

        VL53L4CD_ERROR error;
        // Initialize I2C bus.
        DEV_I2C.begin();

        // Configure VL53L4CD satellite component.
        topVL53L4CDSensor.begin();

        // Switch off VL53L4CD satellite component.
        topVL53L4CDSensor.VL53L4CD_Off();

        // Initialize VL53L4CD satellite component.
        error = topVL53L4CDSensor.InitSensor();
        if (error != 0)
        {
            DEBUG_PRINTF("[%s] VL53L4CD sensor init failed with error code: %d\n", _name, error);
            enabled = false;
            return;
        }

        // Program the highest possible TimingBudget, without enabling the
        // low power mode. This should give the best accuracy
        error = topVL53L4CDSensor.VL53L4CD_SetRangeTiming(200, 0);
        if (error != 0)
        {
            DEBUG_PRINTF("[%s] VL53L4CD sensor set range timing failed with error code: %d\n", _name, error);
            enabled = false;
            return;
        }

        // Start Measurements
        error = topVL53L4CDSensor.VL53L4CD_StartRanging();
        if (error != 0)
        {
            DEBUG_PRINTF("[%s] VL53L4CD sensor start ranging failed with error code: %d\n", _name, error);
            enabled = false;
            return;
        }

        enable(enabled);
        initDone = true;
    }

    void loop()
    {
        if (!enabled || strip.isUpdating())
            return;
        minSegmentId = strip.getMainSegmentId(); // it may not be the best idea to start with main segment as it may not be the first one
        maxSegmentId = strip.getLastActiveSegmentId() + 1;
        checkSensors();
        if (on)
            autoPowerOff();
        updateSwipe();
    }

    uint16_t getId() { return USERMOD_ID_ANIMATED_STAIRCASE; }

#ifndef WLED_DISABLE_MQTT
    /**
     * handling of MQTT message
     * topic only contains stripped topic (part after /wled/MAC)
     * topic should look like: /swipe with amessage of [up|down]
     */
    bool onMqttMessage(char *topic, char *payload)
    {
        if (strlen(topic) == 6 && strncmp_P(topic, PSTR("/swipe"), 6) == 0)
        {
            String action = payload;
            if (action == "up")
            {
                bottomSensorWrite = true;
                return true;
            }
            else if (action == "down")
            {
                topSensorWrite = true;
                return true;
            }
            else if (action == "on")
            {
                enable(true);
                return true;
            }
            else if (action == "off")
            {
                enable(false);
                return true;
            }
        }
        return false;
    }

    /**
     * subscribe to MQTT topic for controlling usermod
     */
    void onMqttConnect(bool sessionPresent)
    {
        //(re)subscribe to required topics
        char subuf[64];
        if (mqttDeviceTopic[0] != 0)
        {
            strcpy(subuf, mqttDeviceTopic);
            strcat_P(subuf, PSTR("/swipe"));
            mqtt->subscribe(subuf, 0);
        }
    }
#endif

    void addToJsonState(JsonObject &root)
    {
        JsonObject staircase = root[FPSTR(_name)];
        if (staircase.isNull())
        {
            staircase = root.createNestedObject(FPSTR(_name));
        }
        writeSensorsToJson(staircase);
        DEBUG_PRINTLN(F("Staircase sensor state exposed in API."));
    }

    /*
     * Reads configuration settings from the json API.
     * See void addToJsonState(JsonObject& root)
     */
    void readFromJsonState(JsonObject &root)
    {
        if (!initDone)
            return; // prevent crash on boot applyPreset()
        bool en = enabled;
        JsonObject staircase = root[FPSTR(_name)];
        if (!staircase.isNull())
        {
            if (staircase[FPSTR(_enabled)].is<bool>())
            {
                en = staircase[FPSTR(_enabled)].as<bool>();
            }
            else
            {
                String str = staircase[FPSTR(_enabled)]; // checkbox -> off or on
                en = (bool)(str != "off");               // off is guaranteed to be present
            }
            if (en != enabled)
                enable(en);
            readSensorsFromJson(staircase);
            DEBUG_PRINTLN(F("Staircase sensor state read from API."));
        }
    }

    void appendConfigData()
    {
        // oappend(_sensorType);
        // oappend(SET_F("dd=addDropdown('staircase','sensor:type');"));
        // oappend(SET_F("addOption(dd,'',0);"));
        // oappend(SET_F("addOption(dd,'PIR',1);"));
        // oappend(SET_F("addOption(dd,'Ultrasound',2);"));
        // oappend(SET_F("addOption(dd,'VL53L4CD',3);"));
    }

    /*
     * Writes the configuration to internal flash memory.
     */
    void addToConfig(JsonObject &root)
    {
        JsonObject staircase = root[FPSTR(_name)];
        if (staircase.isNull())
        {
            staircase = root.createNestedObject(FPSTR(_name));
        }
        staircase[FPSTR(_enabled)] = enabled;
        staircase[FPSTR(_segmentDelay)] = segment_delay_ms;
        staircase[FPSTR(_onTime)] = on_time_ms / 1000;
        staircase[FPSTR(_togglePower)] = togglePower;
        staircase[FPSTR(_topXShutPin)] = topXShutPin;
        staircase[FPSTR(_bottomXShutPin)] = bottomXShutPin;
        DEBUG_PRINTLN(F("Staircase config saved."));
    }

    /*
     * Reads the configuration to internal flash memory before setup() is called.
     *
     * The function should return true if configuration was successfully loaded or false if there was no configuration.
     */
    bool readFromConfig(JsonObject &root)
    {
        int8_t oldTopXShutPin = topXShutPin;
        int8_t oldBottomXShutPin = bottomXShutPin;

        JsonObject top = root[FPSTR(_name)];
        if (top.isNull())
        {
            DEBUG_PRINT(FPSTR(_name));
            DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
            return false;
        }

        enabled = top[FPSTR(_enabled)] | enabled;

        segment_delay_ms = top[FPSTR(_segmentDelay)] | segment_delay_ms;
        segment_delay_ms = (unsigned long)min((unsigned long)10000, max((unsigned long)10, (unsigned long)segment_delay_ms)); // max delay 10s

        on_time_ms = top[FPSTR(_onTime)] | on_time_ms / 1000;
        on_time_ms = min(900, max(10, (int)on_time_ms)) * 1000; // min 10s, max 15min



        topXShutPin = top[FPSTR(_topXShutPin)] | topXShutPin;
        bottomXShutPin = top[FPSTR(_bottomXShutPin)] | bottomXShutPin;

        togglePower = top[FPSTR(_togglePower)] | togglePower; // staircase toggles power on/off

        DEBUG_PRINT(FPSTR(_name));
        if (!initDone)
        {
            // first run: reading from cfg.json
            DEBUG_PRINTLN(F(" config loaded."));
        }
        else
        {
            // changing parameters from settings page
            DEBUG_PRINTLN(F(" config (re)loaded."));
            bool changed = false;
            if ((oldTopXShutPin != topXShutPin) ||
                (oldBottomXShutPin != bottomXShutPin))
            {
                changed = true;
                pinManager.deallocatePin(oldTopXShutPin, PinOwner::UM_AnimatedStaircase);
                pinManager.deallocatePin(oldBottomXShutPin, PinOwner::UM_AnimatedStaircase);
            }
            if (changed)
                setup();
        }
        // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
        return !top[FPSTR(_togglePower)].isNull();
    }

    /*
     * Shows the delay between steps and power-off time in the "info"
     * tab of the web-UI.
     */
    void addToJsonInfo(JsonObject &root)
    {
        JsonObject user = root["u"];
        if (user.isNull())
        {
            user = root.createNestedObject("u");
        }

        JsonArray infoArr = user.createNestedArray(FPSTR(_name)); // name

        String uiDomString = F("<button class=\"btn btn-xs\" onclick=\"requestJson({");
        uiDomString += FPSTR(_name);
        uiDomString += F(":{");
        uiDomString += FPSTR(_enabled);
        uiDomString += enabled ? F(":false}});\">") : F(":true}});\">");
        uiDomString += F("<i class=\"icons ");
        uiDomString += enabled ? "on" : "off";
        uiDomString += F("\">&#xe08f;</i></button>");
        infoArr.add(uiDomString);
    }
};

// strings to reduce flash memory usage (used more than twice)
const char Animated_Staircase_vl53l4cd::_name[] PROGMEM = "staircase";
const char Animated_Staircase_vl53l4cd::_enabled[] PROGMEM = "enabled";
const char Animated_Staircase_vl53l4cd::_segmentDelay[] PROGMEM = "segment-delay-ms";
const char Animated_Staircase_vl53l4cd::_onTime[] PROGMEM = "on-time-s";
const char Animated_Staircase_vl53l4cd::_togglePower[] PROGMEM = "toggle-on-off";
const char Animated_Staircase_vl53l4cd::_topXShutPin[] PROGMEM = "topXShutPin";
const char Animated_Staircase_vl53l4cd::_bottomXShutPin[] PROGMEM = "bottomXShutPin";