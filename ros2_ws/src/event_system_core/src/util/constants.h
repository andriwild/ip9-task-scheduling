#pragma once

constexpr int ONE_HOUR = 3600;
constexpr int SECONDS_PER_DAY = 86400;
constexpr const char* IN_TRANSIT = "IN_TRANSIT";
constexpr const char* OUTDOOR    = "OUTDOOR";

// Order ids are handed out from three ranges, one per source, so a trace shows where an order came from.
constexpr int INTERRUPT_ID_BASE  = 100000;
constexpr int SCHEDULED_ID_BASE  = 200000;
constexpr int BACKGROUND_ID_BASE = 300000;
