#pragma once
#include <Arduino.h>

/*
 * ============================
 * Debug configuration
 * ============================
 */

#define DEBUG_ENABLE 1

#define DEBUG_LEVEL_ERROR 1
#define DEBUG_LEVEL_WARN  2
#define DEBUG_LEVEL_INFO  3
#define DEBUG_LEVEL_DEBUG 4

// Active level
#define DEBUG_LEVEL DEBUG_LEVEL_INFO

// Serial settings
#define DEBUG_BAUD 115200

/*
 * Call once during startup (Robot::init / setup)
 */
#if DEBUG_ENABLE
void debug_init();
#else
inline void debug_init() {}
#endif

/*
 * ============================
 * Basic prints (compile-time removable)
 * ============================
 */
#if DEBUG_ENABLE
    #define DBG_PRINT(x)    Serial.print(x)
    #define DBG_PRINTLN(x)  Serial.println(x)
#else
    #define DBG_PRINT(x)    do {} while (0)
    #define DBG_PRINTLN(x)  do {} while (0)
#endif

/*
 * ============================
 * Level-based logging
 * ============================
 */
#if DEBUG_ENABLE && (DEBUG_LEVEL >= DEBUG_LEVEL_ERROR)
    #define LOG_ERROR(msg)  do { Serial.print("[E] "); Serial.println(msg); } while (0)
#else
    #define LOG_ERROR(msg)  do {} while (0)
#endif

#if DEBUG_ENABLE && (DEBUG_LEVEL >= DEBUG_LEVEL_WARN)
    #define LOG_WARN(msg)   do { Serial.print("[W] "); Serial.println(msg); } while (0)
#else
    #define LOG_WARN(msg)   do {} while (0)
#endif

#if DEBUG_ENABLE && (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
    #define LOG_INFO(msg)   do { Serial.print("[I] "); Serial.println(msg); } while (0)
#else
    #define LOG_INFO(msg)   do {} while (0)
#endif

#if DEBUG_ENABLE && (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG)
    #define LOG_DEBUG(msg)  do { Serial.print("[D] "); Serial.println(msg); } while (0)
#else
    #define LOG_DEBUG(msg)  do {} while (0)
#endif
