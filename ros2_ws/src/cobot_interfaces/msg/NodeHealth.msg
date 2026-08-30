# One node's health as judged by health_monitor (Heartbeat + staleness).
uint8 LEVEL_OK = 0
uint8 LEVEL_WARN = 1
uint8 LEVEL_ERROR = 2
uint8 LEVEL_STALE = 3     # no heartbeat within stale_factor * period_s
uint8 LEVEL_UNKNOWN = 4   # expected node never seen since the monitor started

string node_name
uint8 level
string detail
float32 age_s             # seconds since last heartbeat; -1 if never seen
KeyValue[] metrics        # last known metrics (kept through STALE)
