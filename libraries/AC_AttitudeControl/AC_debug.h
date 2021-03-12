#pragma once

#define _GPC_DEBUG_LOG_05HZ                 (c % 800 == 0)
#define _GPC_DEBUG_LOG_1HZ                  (c % 400 == 0)
#define _GPC_DEBUG_LOG_2HZ                  (c % 200 == 0)
#define _GPC_DEBUG_LOG_5HZ                  (c % 80 == 0)
#define _GPC_HAS_LOGGER                     (_logger != nullptr)

#define GPC_DEBUG_LOG_INIT                  static uint32_t c = 0; c++;
#define GPC_DEBUG_LOG(__fmt__, ...)         _logger->debug_msg(__fmt__, __VA_ARGS__)
#define GPC_DEBUG_LOG_1HZ(__fmt__, ...)     if (_GPC_HAS_LOGGER && _GPC_DEBUG_LOG_1HZ) GPC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define GPC_DEBUG_LOG_2HZ(__fmt__, ...)     if (_GPC_HAS_LOGGER && _GPC_DEBUG_LOG_2HZ) GPC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define GPC_DEBUG_LOG_5HZ(__fmt__, ...)     if (_GPC_HAS_LOGGER && _GPC_DEBUG_LOG_5HZ) GPC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define GPC_DEBUG_LOG_05HZ(__fmt__, ...)    if (_GPC_HAS_LOGGER && _GPC_DEBUG_LOG_05HZ) GPC_DEBUG_LOG(__fmt__, __VA_ARGS__)

#define AC_DEBUG_LOG_INIT                  static uint32_t c = 0; c++;
#define AC_DEBUG_LOG(__fmt__, ...)         gcs().send_text(MAV_SEVERITY_DEBUG, __fmt__, __VA_ARGS__)
#define AC_DEBUG_LOG_1HZ(__fmt__, ...)     if (_GPC_DEBUG_LOG_1HZ) AC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define AC_DEBUG_LOG_2HZ(__fmt__, ...)     if (_GPC_DEBUG_LOG_2HZ) AC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define AC_DEBUG_LOG_5HZ(__fmt__, ...)     if (_GPC_DEBUG_LOG_5HZ) AC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define AC_DEBUG_LOG_05HZ(__fmt__, ...)    if (_GPC_DEBUG_LOG_05HZ) AC_DEBUG_LOG(__fmt__, __VA_ARGS__)