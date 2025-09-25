#include "RcBasicCtr.h"
#include "RcLinkSimple.h"

RcBasicCtr    l2;
RcLinkSimple  link(l2, Serial);

void setup() {
  Serial.begin(115200);
  l2.begin();
  link.begin(50); // telemetry every 50 ms (set 0 to disable)
}

void loop() {
  link.service(); // runs Layer-2 tick + parses commands + telemetry
}
