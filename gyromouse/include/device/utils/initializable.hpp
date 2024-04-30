#pragma once

#include "common.hpp"
#include <FreeRTOS.h>
#include <semphr.h>

class Initializable {
public:
    inline bool is_initialized() { return _is_initialized; };

    Initializable() {
        _initialization_mutex = xSemaphoreCreateMutex();
    }

protected:
    inline void initialize_start() {
        if (_is_initializing) {
            // No need to wait for initialization twice.
            return;
        }

        xSemaphoreTake(_initialization_mutex, portMAX_DELAY);
        _is_initializing = true;
    }
    
    inline void initialize_end() {
        _is_initialized = true;
        xSemaphoreGive(_initialization_mutex);
        _is_initializing = false;
    }
    
private:
    bool _is_initialized = false;
    bool _is_initializing = false;
    SemaphoreHandle_t _initialization_mutex;
};
