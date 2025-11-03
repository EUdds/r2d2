# -*- coding: utf-8 -*-
# Hand-authored protobuf module matching r2bus.proto.

from __future__ import annotations

from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import descriptor_pb2 as _descriptor_pb2
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper

_sym_db = _symbol_database.Default()

_file_descriptor_proto = _descriptor_pb2.FileDescriptorProto()
_file_descriptor_proto.name = "r2bus.proto"
_file_descriptor_proto.package = "r2bus"
_file_descriptor_proto.syntax = "proto3"

_status_enum = _file_descriptor_proto.enum_type.add()
_status_enum.name = "Status"
_status_value = _status_enum.value.add()
_status_value.name = "STATUS_OK"
_status_value.number = 0
_status_value = _status_enum.value.add()
_status_value.name = "STATUS_ERROR"
_status_value.number = 1

_messageid_enum = _file_descriptor_proto.enum_type.add()
_messageid_enum.name = "MessageId"
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_UNKNOWN"
_messageid_value.number = 0
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_ECU_RESET"
_messageid_value.number = 0x01
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_PING"
_messageid_value.number = 0x02
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_PONG"
_messageid_value.number = 0x03
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_HEARTBEAT"
_messageid_value.number = 0x04
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_PSI_COLOR_REQ"
_messageid_value.number = 0x05
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_SERVO_MOVE_CMD"
_messageid_value.number = 0x06
_messageid_value = _messageid_enum.value.add()
_messageid_value.name = "MESSAGE_ID_ACK"
_messageid_value.number = 0x7F

_ack_msg = _file_descriptor_proto.message_type.add()
_ack_msg.name = "Ack"
_field = _ack_msg.field.add()
_field.name = "status"
_field.number = 1
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_ENUM
_field.type_name = ".r2bus.Status"
_field = _ack_msg.field.add()
_field.name = "original_msg"
_field.number = 2
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_ENUM
_field.type_name = ".r2bus.MessageId"

_ping_msg = _file_descriptor_proto.message_type.add()
_ping_msg.name = "Ping"
_field = _ping_msg.field.add()
_field.name = "payload"
_field.number = 1
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_BYTES

_pong_msg = _file_descriptor_proto.message_type.add()
_pong_msg.name = "Pong"
_field = _pong_msg.field.add()
_field.name = "payload"
_field.number = 1
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_BYTES

_heartbeat_msg = _file_descriptor_proto.message_type.add()
_heartbeat_msg.name = "Heartbeat"
_field = _heartbeat_msg.field.add()
_field.name = "uptime_ms"
_field.number = 1
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_UINT32

_psicolor_msg = _file_descriptor_proto.message_type.add()
_psicolor_msg.name = "PsiColorRequest"
_field = _psicolor_msg.field.add()
_field.name = "red"
_field.number = 1
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_UINT32
_field = _psicolor_msg.field.add()
_field.name = "green"
_field.number = 2
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_UINT32
_field = _psicolor_msg.field.add()
_field.name = "blue"
_field.number = 3
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_UINT32

_servo_msg = _file_descriptor_proto.message_type.add()
_servo_msg.name = "ServoMoveCommand"
_field = _servo_msg.field.add()
_field.name = "servo"
_field.number = 1
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_UINT32
_field = _servo_msg.field.add()
_field.name = "position_deg"
_field.number = 2
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_FLOAT
_field = _servo_msg.field.add()
_field.name = "duration_ms"
_field.number = 3
_field.label = _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
_field.type = _descriptor_pb2.FieldDescriptorProto.TYPE_UINT32

_empty_msg = _file_descriptor_proto.message_type.add()
_empty_msg.name = "Empty"

DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(
    _file_descriptor_proto.SerializeToString()
)

_STATUS = DESCRIPTOR.enum_types_by_name["Status"]
Status = _enum_type_wrapper.EnumTypeWrapper(_STATUS)
STATUS_OK = Status.Value("STATUS_OK")
STATUS_ERROR = Status.Value("STATUS_ERROR")

_MESSAGEID = DESCRIPTOR.enum_types_by_name["MessageId"]
MessageId = _enum_type_wrapper.EnumTypeWrapper(_MESSAGEID)
MESSAGE_ID_UNKNOWN = MessageId.Value("MESSAGE_ID_UNKNOWN")
MESSAGE_ID_ECU_RESET = MessageId.Value("MESSAGE_ID_ECU_RESET")
MESSAGE_ID_PING = MessageId.Value("MESSAGE_ID_PING")
MESSAGE_ID_PONG = MessageId.Value("MESSAGE_ID_PONG")
MESSAGE_ID_HEARTBEAT = MessageId.Value("MESSAGE_ID_HEARTBEAT")
MESSAGE_ID_PSI_COLOR_REQ = MessageId.Value("MESSAGE_ID_PSI_COLOR_REQ")
MESSAGE_ID_SERVO_MOVE_CMD = MessageId.Value("MESSAGE_ID_SERVO_MOVE_CMD")
MESSAGE_ID_ACK = MessageId.Value("MESSAGE_ID_ACK")

_ACK = DESCRIPTOR.message_types_by_name["Ack"]
_PING = DESCRIPTOR.message_types_by_name["Ping"]
_PONG = DESCRIPTOR.message_types_by_name["Pong"]
_HEARTBEAT = DESCRIPTOR.message_types_by_name["Heartbeat"]
_PSICOLORREQUEST = DESCRIPTOR.message_types_by_name["PsiColorRequest"]
_SERVOMOVECOMMAND = DESCRIPTOR.message_types_by_name["ServoMoveCommand"]
_EMPTY = DESCRIPTOR.message_types_by_name["Empty"]


class Ack(_message.Message, metaclass=_reflection.GeneratedProtocolMessageType):
    DESCRIPTOR = _ACK
    __module__ = __name__


class Ping(_message.Message, metaclass=_reflection.GeneratedProtocolMessageType):
    DESCRIPTOR = _PING
    __module__ = __name__


class Pong(_message.Message, metaclass=_reflection.GeneratedProtocolMessageType):
    DESCRIPTOR = _PONG
    __module__ = __name__


class Heartbeat(_message.Message, metaclass=_reflection.GeneratedProtocolMessageType):
    DESCRIPTOR = _HEARTBEAT
    __module__ = __name__


class PsiColorRequest(_message.Message, metaclass=_reflection.GeneratedProtocolMessageType):
    DESCRIPTOR = _PSICOLORREQUEST
    __module__ = __name__


class ServoMoveCommand(_message.Message, metaclass=_reflection.GeneratedProtocolMessageType):
    DESCRIPTOR = _SERVOMOVECOMMAND
    __module__ = __name__


class Empty(_message.Message, metaclass=_reflection.GeneratedProtocolMessageType):
    DESCRIPTOR = _EMPTY
    __module__ = __name__


_sym_db.RegisterMessage(Ack)
_sym_db.RegisterMessage(Ping)
_sym_db.RegisterMessage(Pong)
_sym_db.RegisterMessage(Heartbeat)
_sym_db.RegisterMessage(PsiColorRequest)
_sym_db.RegisterMessage(ServoMoveCommand)
_sym_db.RegisterMessage(Empty)

__all__ = [
    "Ack",
    "Ping",
    "Pong",
    "Heartbeat",
    "PsiColorRequest",
    "ServoMoveCommand",
    "Empty",
    "Status",
    "STATUS_OK",
    "STATUS_ERROR",
    "MessageId",
    "MESSAGE_ID_UNKNOWN",
    "MESSAGE_ID_ECU_RESET",
    "MESSAGE_ID_PING",
    "MESSAGE_ID_PONG",
    "MESSAGE_ID_HEARTBEAT",
    "MESSAGE_ID_PSI_COLOR_REQ",
    "MESSAGE_ID_SERVO_MOVE_CMD",
    "MESSAGE_ID_ACK",
]
