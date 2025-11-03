/* Hand-authored nanopb descriptor bindings for r2bus.proto. */

#include "r2bus.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

PB_BIND(r2bus_Ack, r2bus_Ack, AUTO)
PB_BIND(r2bus_Ping, r2bus_Ping, AUTO)
PB_BIND(r2bus_Pong, r2bus_Pong, AUTO)
PB_BIND(r2bus_Heartbeat, r2bus_Heartbeat, AUTO)
PB_BIND(r2bus_PsiColorRequest, r2bus_PsiColorRequest, AUTO)
PB_BIND(r2bus_ServoMoveCommand, r2bus_ServoMoveCommand, AUTO)
PB_BIND(r2bus_Empty, r2bus_Empty, AUTO)
