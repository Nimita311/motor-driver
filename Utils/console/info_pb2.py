# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: info.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import nanopb_pb2 as nanopb__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='info.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=b'\n\ninfo.proto\x1a\x0cnanopb.proto\"N\n\x04Info\x12\x11\n\ttimestamp\x18\x01 \x01(\r\x12\n\n\x02id\x18\x02 \x01(\r\x12\x1c\n\x08pid_info\x18\x03 \x01(\x0b\x32\x08.PIDInfoH\x00\x42\t\n\x07\x63ontent\"v\n\x07PIDInfo\x12\t\n\x01\x65\x18\x01 \x01(\x02\x12\x0e\n\x06\x65_real\x18\x02 \x01(\x02\x12\t\n\x01x\x18\x03 \x01(\x02\x12\x0e\n\x06x_real\x18\x04 \x01(\x02\x12\t\n\x01y\x18\x05 \x01(\x02\x12\x0e\n\x06y_real\x18\x06 \x01(\x02\x12\x1a\n\x12\x61nti_windup_active\x18\x07 \x01(\x08\"f\n\x07\x43PUInfo\x12+\n\ttask_info\x18\x01 \x03(\x0b\x32\x11.CPUInfo.TaskInfoB\x05\x92?\x02\x10\x05\x1a.\n\x08TaskInfo\x12\x13\n\x04name\x18\x01 \x01(\tB\x05\x92?\x02\x08\x10\x12\r\n\x05usage\x18\x02 \x01(\rb\x06proto3'
  ,
  dependencies=[nanopb__pb2.DESCRIPTOR,])




_INFO = _descriptor.Descriptor(
  name='Info',
  full_name='Info',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='Info.timestamp', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='id', full_name='Info.id', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pid_info', full_name='Info.pid_info', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='content', full_name='Info.content',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=28,
  serialized_end=106,
)


_PIDINFO = _descriptor.Descriptor(
  name='PIDInfo',
  full_name='PIDInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='e', full_name='PIDInfo.e', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='e_real', full_name='PIDInfo.e_real', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='x', full_name='PIDInfo.x', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='x_real', full_name='PIDInfo.x_real', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='PIDInfo.y', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y_real', full_name='PIDInfo.y_real', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='anti_windup_active', full_name='PIDInfo.anti_windup_active', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=108,
  serialized_end=226,
)


_CPUINFO_TASKINFO = _descriptor.Descriptor(
  name='TaskInfo',
  full_name='CPUInfo.TaskInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='CPUInfo.TaskInfo.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\002\010\020', file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='usage', full_name='CPUInfo.TaskInfo.usage', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=284,
  serialized_end=330,
)

_CPUINFO = _descriptor.Descriptor(
  name='CPUInfo',
  full_name='CPUInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='task_info', full_name='CPUInfo.task_info', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\002\020\005', file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_CPUINFO_TASKINFO, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=228,
  serialized_end=330,
)

_INFO.fields_by_name['pid_info'].message_type = _PIDINFO
_INFO.oneofs_by_name['content'].fields.append(
  _INFO.fields_by_name['pid_info'])
_INFO.fields_by_name['pid_info'].containing_oneof = _INFO.oneofs_by_name['content']
_CPUINFO_TASKINFO.containing_type = _CPUINFO
_CPUINFO.fields_by_name['task_info'].message_type = _CPUINFO_TASKINFO
DESCRIPTOR.message_types_by_name['Info'] = _INFO
DESCRIPTOR.message_types_by_name['PIDInfo'] = _PIDINFO
DESCRIPTOR.message_types_by_name['CPUInfo'] = _CPUINFO
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Info = _reflection.GeneratedProtocolMessageType('Info', (_message.Message,), {
  'DESCRIPTOR' : _INFO,
  '__module__' : 'info_pb2'
  # @@protoc_insertion_point(class_scope:Info)
  })
_sym_db.RegisterMessage(Info)

PIDInfo = _reflection.GeneratedProtocolMessageType('PIDInfo', (_message.Message,), {
  'DESCRIPTOR' : _PIDINFO,
  '__module__' : 'info_pb2'
  # @@protoc_insertion_point(class_scope:PIDInfo)
  })
_sym_db.RegisterMessage(PIDInfo)

CPUInfo = _reflection.GeneratedProtocolMessageType('CPUInfo', (_message.Message,), {

  'TaskInfo' : _reflection.GeneratedProtocolMessageType('TaskInfo', (_message.Message,), {
    'DESCRIPTOR' : _CPUINFO_TASKINFO,
    '__module__' : 'info_pb2'
    # @@protoc_insertion_point(class_scope:CPUInfo.TaskInfo)
    })
  ,
  'DESCRIPTOR' : _CPUINFO,
  '__module__' : 'info_pb2'
  # @@protoc_insertion_point(class_scope:CPUInfo)
  })
_sym_db.RegisterMessage(CPUInfo)
_sym_db.RegisterMessage(CPUInfo.TaskInfo)


_CPUINFO_TASKINFO.fields_by_name['name']._options = None
_CPUINFO.fields_by_name['task_info']._options = None
# @@protoc_insertion_point(module_scope)
