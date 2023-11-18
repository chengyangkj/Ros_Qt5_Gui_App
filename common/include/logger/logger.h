#pragma once
#include "easylogging++.h"
#ifndef LOGGER_INFO
#define LOGGER_INFO(str) LOG(INFO) << str;
#endif
#ifndef LOGGER_ERROR
#define LOGGER_ERROR(str) LOG(ERROR) << str;
#endif
#ifndef LOGGER_WARN
#define LOGGER_WARN(str) LOG(WARN) << str;
#endif