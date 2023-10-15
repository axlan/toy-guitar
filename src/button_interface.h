#pragma once

#include <cstdint>

void InitButtonPins();

uint8_t ReadButtons();

uint8_t GetLowestPressed();
