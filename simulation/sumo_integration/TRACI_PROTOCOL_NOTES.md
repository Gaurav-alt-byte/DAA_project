# TraCI wire protocol — worked decode

This is the ground truth `traci_client.h` is implemented against, captured
by monkey-patching `socket.sendall`/`recv` inside the official Python
`traci` client talking to a real `sumo 1.27.1` process, then decoding the
bytes with `traci`'s own `Storage` class (not by hand) to eliminate
guesswork. Reproduce with `/tmp/dump.py`-style capture if the protocol
ever needs re-verifying against a newer SUMO version.

## Message framing
```
[4-byte big-endian length, INCLUDING these 4 bytes][body]
```

## Command framing (one or more back-to-back inside a message body)
```
[1-byte length, INCLUDING this byte][cmdId][payload]
```
If a sub-command would be >= 255 bytes, the length byte is `0x00`
followed by a 4-byte big-endian extended length (seen on the *receive*
side once a vehicle-id-list answer gets long).

## Strings
```
[4-byte big-endian length][UTF-8 bytes], no terminator
```

## Status reply (one per command sent, always present)
```
[len][cmdId][resultCode][errorString]
```
`resultCode == 0` means success; `errorString` is empty on success.

## GET_* answer (only for GET-style commands, follows the status)
```
[len][cmdId + 0x10][variableId][objectIdString][valueType][value...]
```
Value encodings actually used here:
- `TYPE_STRINGLIST` (0x0e): `[int32 count][string]*count`
- `POSITION_2D` (0x01): `[double x][double y]`
- `TYPE_DOUBLE` (0x0b): `[double]`

## getVersion is a special case
No answer wrapper — after the status, the remaining bytes are just
`[length byte][int32 apiVersion][string versionString]` directly (no
repeated command id). Not used by `traci_client.h` (we don't need the
handshake), but documented here since it's what stalled the previous
attempt at this integration.

## CMD_SIMSTEP (0x02)
Request payload: `[double targetTime]` — `0.0` means "advance exactly
one step". Response is just the status, with `[int32 subscriptionCount]`
appended (0 here, since we never issue TraCI subscriptions — every GET
is a fresh polled command instead of a persistent subscription).

## Worked example (all captured live, hex)
```
getVehicleIDList request:  0000000b07a40000000000
getVehicleIDList response: 00000017 07a400000000000cb400000000000e00000000
  -> status: cmdLen=7 cmd=0xa4 result=0 err=''
  -> answer: len=12 respCmd=0xb4 varId=0x00 objId='' valType=0x0e (STRINGLIST) n=0

getPosition request:  0000000f0ba4420000000476656830
getPosition response: 00000027 07a400000000001cb4420000000476656830014079f800000000004068cccccccccccd
  -> status: cmdLen=7 cmd=0xa4 result=0 err=''
  -> answer: len=28 respCmd=0xb4 varId=0x42 objId='veh0' valType=0x01 (POSITION_2D) -> (415.5, 198.4)
```
