# Copyright 2015-2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import OrderedDict
import os
import re
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/bin")

import ament_index_python
from catkin_pkg.package import parse_package

import msgloader
import msgloader.msg_loader

import rosidl_adapter.parser
from rosidl_cmake import expand_template
import rosidl_parser.parser

import yaml

# more MROS imports
# since ROS should be sourced after MROS it will overlay
# e.g. the message packages, to prevent that from happening (and MROS code to
# fail) MROS paths are moved to the front of the search path
# rpp = os.environ.get('ROS_PACKAGE_PATH', '').split(os.pathsep)
# for package_path in reversed([p for p in rpp if p]):
#     if not package_path.endswith(os.sep + 'share'):
#         continue
#     mros_basepath = os.path.dirname(package_path)
#     for sys_path in sys.path:
#         if sys_path.startswith(os.path.join(mros_basepath, '')):
#             sys.path.remove(sys_path)
#             sys.path.insert(0, sys_path)
# import rosmsg  # noqa


def generate_cpp(output_path, template_dir, msgs_dir):
    data = generate_messages(msgs_dir)
    message_string_pairs = {
        (
            '%s/%s' % (m.mros_msg.package_name, m.mros_msg.message_name),
            '%s/%s' % (m.ros_msg.package_name, m.ros_msg.message_name))
        for m in data['mappings']}
    data.update(
        generate_services(msgs_dir, message_string_pairs=message_string_pairs))

    template_file = os.path.join(template_dir, 'get_mappings.cpp.em')
    output_file = os.path.join(output_path, 'get_mappings.cpp')
    data_for_template = {
        'mappings': data['mappings'], 'services': data['services']}
    expand_template(template_file, data_for_template, output_file)

    unique_package_names = set(data['ros_package_names_msg'] + data['ros_package_names_srv'])
    # skip builtin_interfaces since there is a custom implementation
    unique_package_names -= {'builtin_interfaces'}
    data['ros_package_names'] = list(unique_package_names)

    template_file = os.path.join(template_dir, 'get_factory.cpp.em')
    output_file = os.path.join(output_path, 'get_factory.cpp')
    expand_template(template_file, data, output_file)

    for ros_package_name in data['ros_package_names']:
        data_pkg_hpp = {
            'ros_package_name': ros_package_name,
            # include directives and template types
            'mapped_mros_msgs': [
                m.mros_msg for m in data['mappings']
                if m.ros_msg.package_name == ros_package_name],
            'mapped_ros_msgs': [
                m.ros_msg for m in data['mappings']
                if m.ros_msg.package_name == ros_package_name],
            # forward declaration of factory functions
            'ros_msg_types': [
                m for m in data['all_ros_msgs']
                if m.package_name == ros_package_name],
            'ros_srv_types': [
                s for s in data['all_ros_srvs']
                if s.package_name == ros_package_name],
            # forward declaration of template specializations
            'mappings': [
                m for m in data['mappings']
                if m.ros_msg.package_name == ros_package_name],
        }
        template_file = os.path.join(template_dir, 'pkg_factories.hpp.em')
        output_file = os.path.join(
            output_path, '%s_factories.hpp' % ros_package_name)
        expand_template(template_file, data_pkg_hpp, output_file)

        data_pkg_cpp = {
            'ros_package_name': ros_package_name,
            # call interface specific factory functions
            'ros_msg_types': data_pkg_hpp['ros_msg_types'],
            'ros_srv_types': data_pkg_hpp['ros_srv_types'],
        }
        template_file = os.path.join(template_dir, 'pkg_factories.cpp.em')
        output_file = os.path.join(
            output_path, '%s_factories.cpp' % ros_package_name)
        expand_template(template_file, data_pkg_cpp, output_file)

        for interface_type, interfaces in zip(
            ['msg', 'srv'], [data['all_ros_msgs'], data['all_ros_srvs']]
        ):
            for interface in interfaces:
                if interface.package_name != ros_package_name:
                    continue
                data_idl_cpp = {
                    'ros_package_name': ros_package_name,
                    'interface_type': interface_type,
                    'interface': interface,
                    'mapped_msgs': [],
                    'mapped_services': [],
                }
                if interface_type == 'msg':
                    data_idl_cpp['mapped_msgs'] += [
                        m for m in data['mappings']
                        if m.ros_msg.package_name == ros_package_name and
                        m.ros_msg.message_name == interface.message_name]
                if interface_type == 'srv':
                    data_idl_cpp['mapped_services'] += [
                        s for s in data['services']
                        if s['ros_package'] == ros_package_name and
                        s['ros_name'] == interface.message_name]
                template_file = os.path.join(template_dir, 'interface_factories.cpp.em')
                output_file = os.path.join(
                    output_path, '%s__%s__%s__factories.cpp' %
                    (ros_package_name, interface_type, interface.message_name))
                expand_template(template_file, data_idl_cpp, output_file)


def generate_messages(msgs_dir):
    mros_msgs = get_mros_messages(msgs_dir)
    ros_package_names, ros_msgs, mapping_rules = get_ros_messages()

    package_pairs = determine_package_pairs(mros_msgs, ros_msgs, mapping_rules)
    message_pairs = determine_message_pairs(mros_msgs, ros_msgs, package_pairs, mapping_rules)

    mappings = []
    # add custom mapping for builtin_interfaces
    for msg_name in ('Duration', 'Time'):
        mros_msg = [
            m for m in mros_msgs
            if m.package_name == 'std_msgs' and m.message_name == msg_name]
        ros_msg = [
            m for m in ros_msgs
            if m.package_name == 'builtin_interfaces' and m.message_name == msg_name]
        if mros_msg and ros_msg:
            mappings.append(Mapping(mros_msg[0], ros_msg[0]))

    msg_idx = MessageIndex()
    for mros_msg, ros_msg in message_pairs:
        msg_idx.mros_put(mros_msg)
        msg_idx.ros_put(ros_msg)

    for mros_msg, ros_msg in message_pairs:
        mapping = determine_field_mapping(mros_msg, ros_msg, mapping_rules, msg_idx)
        if mapping:
            mappings.append(mapping)

    # order mappings topologically to allow template specialization
    ordered_mappings = []
    while mappings:
        # pick first mapping without unsatisfied dependencies
        for m in mappings:
            if not m.depends_on_ros_messages:
                break
        else:
            break
        # move mapping to ordered list
        mappings.remove(m)
        ordered_mappings.append(m)
        ros_msg = m.ros_msg
        # update unsatisfied dependencies of remaining mappings
        for m in mappings:
            if ros_msg in m.depends_on_ros_messages:
                m.depends_on_ros_messages.remove(ros_msg)

    if mappings:
        print('%d mappings can not be generated due to missing dependencies:' % len(mappings),
              file=sys.stderr)
        for m in mappings:
            print('- %s <-> %s:' %
                  ('%s/%s' % (m.mros_msg.package_name, m.mros_msg.message_name),
                   '%s/%s' % (m.ros_msg.package_name, m.ros_msg.message_name)), file=sys.stderr)
            for d in m.depends_on_ros_messages:
                print('  -', '%s/%s' % (d.package_name, d.message_name), file=sys.stderr)
        print(file=sys.stderr)

    return {
        'mros_msgs': [m.mros_msg for m in ordered_mappings],
        'ros_msgs': [m.ros_msg for m in ordered_mappings],
        'mappings': ordered_mappings,
        'ros_package_names_msg': ros_package_names,
        'all_ros_msgs': ros_msgs,
    }


def generate_services(msgs_dir, message_string_pairs=None):
    mros_srvs = get_mros_services(msgs_dir)
    ros_pkgs, ros_srvs, mapping_rules = get_ros_services()
    services = determine_common_services(
        mros_srvs, ros_srvs, mapping_rules,
        message_string_pairs=message_string_pairs)
    return {
        'services': services,
        'ros_package_names_srv': ros_pkgs,
        'all_ros_srvs': ros_srvs,
    }

def get_mros_messages(msgs_dir):
    mros_msgs = []
    mros_pkgs = []
    msgs_path = msgs_dir
    for pkg in sorted(os.listdir(msgs_path)):
        pkg_path = msgs_path + "/" + pkg + "/msg"
        mros_pkgs.append((pkg, pkg_path))
    for package_name, path in mros_pkgs:
        if os.path.exists(path):
            for f in os.listdir(path):
                if f.endswith(".msg"):
                    mros_msgs.append(Message(package_name, f[0:-4], path))
    return mros_msgs

def get_ros_messages():
    pkgs = []
    msgs = []
    rules = []
    # get messages from packages
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
        pkgs.append(package_name)
        resource, _ = ament_index_python.get_resource(resource_type, package_name)
        interfaces = resource.splitlines()
        message_names = {
            i[4:-4]
            for i in interfaces
            if i.startswith('msg/') and i[-4:] in ('.idl', '.msg')}

        for message_name in sorted(message_names):
            msgs.append(Message(package_name, message_name, prefix_path))
        # check package manifest for mapping rules
        package_path = os.path.join(prefix_path, 'share', package_name)
        pkg = parse_package(package_path)
        for export in pkg.exports:
            if export.tagname != 'mrosbridger':
                continue
            if 'mapping_rules' not in export.attributes:
                continue
            rule_file = os.path.join(package_path, export.attributes['mapping_rules'])
            with open(rule_file, 'r') as h:
                content = yaml.safe_load(h)
            if not isinstance(content, list):
                print(
                    "The content of the mapping rules in '%s' is not a list" % rule_file,
                    file=sys.stderr)
                continue
            for data in content:
                if all(n not in data for n in ('mros_service_name', 'ros_service_name')):
                    try:
                        rules.append(MessageMappingRule(data, package_name))
                    except Exception as e:
                        print('%s' % str(e), file=sys.stderr)
    return pkgs, msgs, rules

def get_mros_services(msgs_dir):
    mros_srvs = []
    mros_pkgs = []
    msgs_path = msgs_dir
    for pkg in sorted(os.listdir(msgs_path)):
        pkg_path = msgs_path + "/" + pkg + "/srv"
        mros_pkgs.append((pkg, pkg_path))
    for package_name, path in mros_pkgs:
        if os.path.exists(path):
            for f in os.listdir(path):
                if f.endswith(".srv"):
                    mros_srvs.append(Message(package_name, f[0:-4], path))
    return mros_srvs


def get_ros_services():
    pkgs = []
    srvs = []
    rules = []
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
        pkgs.append(package_name)
        resource, _ = ament_index_python.get_resource(resource_type, package_name)
        interfaces = resource.splitlines()
        service_names = {
            i[4:-4]
            for i in interfaces
            if i.startswith('srv/') and i[-4:] in ('.idl', '.srv')}

        for service_name in sorted(service_names):
            srvs.append(Message(package_name, service_name, prefix_path))
        # check package manifest for mapping rules
        package_path = os.path.join(prefix_path, 'share', package_name)
        pkg = parse_package(package_path)
        for export in pkg.exports:
            if export.tagname != 'mrosbridger':
                continue
            if 'mapping_rules' not in export.attributes:
                continue
            rule_file = os.path.join(package_path, export.attributes['mapping_rules'])
            with open(rule_file, 'r') as h:
                content = yaml.safe_load(h)
            if not isinstance(content, list):
                print(
                    "The content of the mapping rules in '%s' is not a list" % rule_file,
                    file=sys.stderr)
                continue
            for data in content:
                if all(n not in data for n in ('mros_message_name', 'ros_message_name')):
                    try:
                        rules.append(ServiceMappingRule(data, package_name))
                    except Exception as e:
                        print('%s' % str(e), file=sys.stderr)
    return pkgs, srvs, rules


class Message:
    __slots__ = [
        'package_name',
        'message_name',
        'prefix_path'
    ]

    def __init__(self, package_name, message_name, prefix_path=None):
        self.package_name = package_name
        self.message_name = message_name
        self.prefix_path = prefix_path

    def __eq__(self, other):
        return self.package_name == other.package_name and \
            self.message_name == other.message_name

    def __hash__(self):
        return hash('%s/%s' % (self.package_name, self.message_name))

    def __str__(self):
        return self.prefix_path + ':' + self.package_name + ':' + self.message_name

    def __repr__(self):
        return self.__str__()


class MappingRule:
    __slots__ = [
        'mros_package_name',
        'ros_package_name',
        'package_mapping'
    ]

    def __init__(self, data, expected_package_name):
        if all(n in data for n in ('mros_package_name', 'ros_package_name')):
            if data['ros_package_name'] != expected_package_name:
                raise Exception(
                    ('Ignoring rule which affects a different ROS package (%s) '
                     'then the one it is defined in (%s)') %
                    (data['ros_package_name'], expected_package_name))
            self.mros_package_name = data['mros_package_name']
            self.ros_package_name = data['ros_package_name']
            self.package_mapping = (len(data) == 2)
        else:
            raise Exception('Ignoring a rule without a mros_package_name and/or ros_package_name')

    def is_package_mapping(self):
        return self.package_mapping

    def __repr__(self):
        return self.__str__()


class MessageMappingRule(MappingRule):
    __slots__ = [
        'mros_message_name',
        'ros_message_name',
        'fields_mros_to_ros',
    ]

    def __init__(self, data, expected_package_name):
        super().__init__(data, expected_package_name)
        self.mros_message_name = None
        self.ros_message_name = None
        self.fields_mros_to_ros = None
        if all(n in data for n in ('mros_message_name', 'ros_message_name')):
            self.mros_message_name = data['mros_message_name']
            self.ros_message_name = data['ros_message_name']
            if 'fields_mros_to_ros' in data:
                self.fields_mros_to_ros = OrderedDict()
                for mros_field_name, ros_field_name in data['fields_mros_to_ros'].items():
                    self.fields_mros_to_ros[mros_field_name] = ros_field_name
            elif len(data) > 4:
                raise RuntimeError(
                    'Mapping for package %s contains unknown field(s)' % self.ros_package_name)
        elif len(data) > 2:
            raise RuntimeError(
                'Mapping for package %s contains unknown field(s)' % self.ros_package_name)

    def is_message_mapping(self):
        return self.mros_message_name is not None

    def is_field_mapping(self):
        return self.fields_mros_to_ros is not None

    def __str__(self):
        return 'MessageMappingRule(%s <-> %s)' % (self.mros_package_name, self.ros_package_name)


class ServiceMappingRule(MappingRule):
    __slots__ = [
        'mros_service_name',
        'ros_service_name',
        'request_fields_1_to_2',
        'response_fields_1_to_2',
    ]

    def __init__(self, data, expected_package_name):
        super().__init__(data, expected_package_name)
        self.mros_service_name = None
        self.ros_service_name = None
        self.request_fields_1_to_2 = None
        self.response_fields_1_to_2 = None
        if all(n in data for n in ('mros_service_name', 'ros_service_name')):
            self.mros_service_name = data['mros_service_name']
            self.ros_service_name = data['ros_service_name']
            expected_keys = 4
            if 'request_fields_1_to_2' in data:
                self.request_fields_1_to_2 = OrderedDict()
                for mros_field_name, ros_field_name in data['request_fields_1_to_2'].items():
                    self.request_fields_1_to_2[mros_field_name] = ros_field_name
                expected_keys += 1
            if 'response_fields_1_to_2' in data:
                self.response_fields_1_to_2 = OrderedDict()
                for mros_field_name, ros_field_name in data['response_fields_1_to_2'].items():
                    self.response_fields_1_to_2[mros_field_name] = ros_field_name
                expected_keys += 1
            elif len(data) > expected_keys:
                raise RuntimeError(
                    'Mapping for package %s contains unknown field(s)' % self.ros_package_name)
        elif len(data) > 2:
            raise RuntimeError(
                'Mapping for package %s contains unknown field(s)' % self.ros_package_name)

    def __str__(self):
        return 'ServiceMappingRule(%s <-> %s)' % (self.mros_package_name, self.ros_package_name)


def determine_package_pairs(mros_msgs, ros_msgs, mapping_rules):
    pairs = []
    # determine package names considered equal between MROS and ROS
    mros_suffix = '_msgs'
    ros_suffixes = ['_msgs', '_interfaces']
    mros_package_names = {m.package_name for m in mros_msgs}
    ros_package_names = {m.package_name for m in ros_msgs}
    for mros_package_name in mros_package_names:
        if not mros_package_name.endswith(mros_suffix):
            continue
        mros_package_basename = mros_package_name[:-len(mros_suffix)]

        for ros_package_name in ros_package_names:
            for ros_suffix in ros_suffixes:
                if ros_package_name.endswith(ros_suffix):
                    break
            else:
                continue
            ros_package_basename = ros_package_name[:-len(ros_suffix)]
            if mros_package_basename != ros_package_basename:
                continue
            pairs.append((mros_package_name, ros_package_name))

    # add manual package mapping rules
    for rule in mapping_rules:
        if not rule.is_package_mapping():
            continue
        if rule.mros_package_name not in mros_package_names:
            continue
        if rule.ros_package_name not in ros_package_names:
            continue
        pair = (rule.mros_package_name, rule.ros_package_name)
        if pair not in pairs:
            pairs.append(pair)

    return pairs


def determine_message_pairs(mros_msgs, ros_msgs, package_pairs, mapping_rules):
    pairs = []
    # determine message names considered equal between MROS and ROS
    for mros_msg in mros_msgs:
        for ros_msg in ros_msgs:
            package_pair = (mros_msg.package_name, ros_msg.package_name)
            if package_pair not in package_pairs:
                continue
            if mros_msg.message_name != ros_msg.message_name:
                continue
            pairs.append((mros_msg, ros_msg))

    # add manual message mapping rules
    for rule in mapping_rules:
        if not rule.is_message_mapping():
            continue
        for mros_msg in mros_msgs:
            if rule.mros_package_name == mros_msg.package_name and \
                    rule.mros_message_name == mros_msg.message_name:
                break
        else:
            # skip unknown messages
            continue
        for ros_msg in ros_msgs:
            if rule.ros_package_name == ros_msg.package_name and \
                    rule.ros_message_name == ros_msg.message_name:
                break
        else:
            # skip unknown messages
            continue

        pair = (mros_msg, ros_msg)
        if pair not in pairs:
            pairs.append(pair)

    return pairs


def determine_common_services(
    mros_srvs, ros_srvs, mapping_rules, message_string_pairs=None
):
    if message_string_pairs is None:
        message_string_pairs = set()

    pairs = []
    services = []
    for mros_srv in mros_srvs:
        for ros_srv in ros_srvs:
            if mros_srv.package_name == ros_srv.package_name:
                if mros_srv.message_name == ros_srv.message_name:
                    pairs.append((mros_srv, ros_srv))

    for rule in mapping_rules:
        for mros_srv in mros_srvs:
            for ros_srv in ros_srvs:
                pair = (mros_srv, ros_srv)
                if pair in pairs:
                    continue
                if rule.mros_package_name == mros_srv.package_name and \
                   rule.ros_package_name == ros_srv.package_name:
                    if rule.mros_service_name is None and rule.ros_service_name is None:
                        if mros_srv.message_name == ros_srv.message_name:
                            pairs.append(pair)
                    else:
                        if (
                            rule.mros_service_name == mros_srv.message_name and
                            rule.ros_service_name == ros_srv.message_name
                        ):
                            pairs.append(pair)

    for pair in pairs:
        mros_spec = load_mros_service(pair[0])
        ros_spec = load_ros_service(pair[1])
        mros_fields = {
            'request': mros_spec.request.fields(),
            'response': mros_spec.response.fields()
        }
        ros_fields = {
            'request': ros_spec.request.fields,
            'response': ros_spec.response.fields
        }
        output = {
            'request': [],
            'response': []
        }
        match = True
        for direction in ['request', 'response']:
            if len(mros_fields[direction]) != len(ros_fields[direction]):
                match = False
                break
            for i, mros_field in enumerate(mros_fields[direction]):
                mros_type = mros_field[0]
                ros_type = str(ros_fields[direction][i].type)
                mros_name = mros_field[1]
                ros_name = ros_fields[direction][i].name
                if mros_type != ros_type or mros_name != ros_name:
                    # if the message types have a custom mapping their names
                    # might not be equal, therefore check the message pairs
                    if (mros_type, ros_type) not in message_string_pairs:
                        match = False
                        break
                output[direction].append({
                    'basic': False if '/' in mros_type else True,
                    'array': True if '[]' in mros_type else False,
                    'mros': {
                        'name': mros_name,
                        'type': mros_type.rstrip('[]'),
                        'cpptype': mros_type.rstrip('[]').replace('/', '::')
                    },
                    'ros': {
                        'name': ros_name,
                        'type': ros_type.rstrip('[]'),
                        'cpptype': ros_type.rstrip('[]').replace('/', '::msg::')
                    }
                })
        if match:
            services.append({
                'mros_name': pair[0].message_name,
                'ros_name': pair[1].message_name,
                'mros_package': pair[0].package_name,
                'ros_package': pair[1].package_name,
                'fields': output
            })
    return services


def update_mros_field_information(mros_field, package_name):
    parts = mros_field.base_type.split('/')
    assert len(parts) in [1, 2]
    if len(parts) == 1:
        mros_field.pkg_name = package_name
        mros_field.msg_name = parts[0]
    else:
        mros_field.pkg_name = parts[0]
        mros_field.msg_name = parts[1]


def get_mros_selected_fields(mros_field_selection, parent_mros_spec, msg_idx):
    """
    Get a tuple of fields corresponding to a field selection on a MROS message.

    :param mros_field_selection: a string with message field names separated by `.`
    :param parent_mros_spec: a msgloader.MsgSpec for a message that contains the first field
    in mros_field_selection
    :type msg_idx: MessageIndex

    :return: a tuple of msgloader.msgs.Field objets with additional attributes `pkg_name`
    and `msg_name` as defined by `update_mros_field_information`, corresponding to
    traversing `parent_mros_spec` recursively following `mros_field_selection`

    :throws: IndexError in case some expected field is not found while traversing
    `parent_mros_spec` recursively following `mros_field_selection`
    """
    selected_fields = []

    def consume_field(field):
        update_mros_field_information(field, parent_mros_spec.package)
        selected_fields.append(field)

    fields = mros_field_selection.split('.')
    current_field = [f for f in parent_mros_spec.parsed_fields() if f.name == fields[0]][0]
    consume_field(current_field)
    for field in fields[1:]:
        parent_mros_spec = load_mros_message(msg_idx.mros_get_from_field(current_field))
        current_field = [f for f in parent_mros_spec.parsed_fields() if f.name == field][0]
        consume_field(current_field)

    return tuple(selected_fields)


def get_ros_selected_fields(ros_field_selection, parent_ros_spec, msg_idx):
    selected_fields = []
    fields = ros_field_selection.split('.')
    current_field = [
        member for member in parent_ros_spec.structure.members
        if member.name == fields[0]
    ][0]
    selected_fields.append(current_field)
    for field in fields[1:]:
        parent_ros_spec = load_ros_message(msg_idx.ros_get_from_field(current_field))
        current_field = [
            member for member in parent_ros_spec.structure.members
            if member.name == field
        ][0]
        selected_fields.append(current_field)
    return tuple(selected_fields)


def determine_field_mapping(mros_msg, ros_msg, mapping_rules, msg_idx):
    """
    Return the first mapping object for mros_msg and ros_msg found in mapping_rules.

    If not found in mapping_rules otherwise defined implicitly, or None if no mapping is found.

    :type mros_msg: Message
    :type ros_msg: Message
    :type mapping_rules: list of MessageMappingRule
    :type msg_idx: MessageIndex
    """
    mros_spec = load_mros_message(mros_msg)
    if not mros_spec:
        return None
    ros_spec = load_ros_message(ros_msg)
    if not ros_spec:
        return None

    mapping = Mapping(mros_msg, ros_msg)

    # check for manual field mapping rules first
    for rule in mapping_rules:
        if not rule.is_field_mapping():
            continue
        if rule.mros_package_name != mros_msg.package_name or \
                rule.mros_message_name != mros_msg.message_name:
            continue
        if rule.ros_package_name != ros_msg.package_name or \
                rule.ros_message_name != ros_msg.message_name:
            continue

        for mros_field_selection, ros_field_selection in rule.fields_mros_to_ros.items():
            try:
                mros_selected_fields = \
                    get_mros_selected_fields(mros_field_selection, mros_spec, msg_idx)
            except IndexError:
                print(
                    "A manual mapping refers to an invalid field '%s' " % mros_field_selection +
                    "in the MROS message '%s/%s'" %
                    (rule.mros_package_name, rule.mros_message_name),
                    file=sys.stderr)
                continue
            try:
                ros_selected_fields = \
                    get_ros_selected_fields(ros_field_selection, ros_spec, msg_idx)
            except IndexError:
                print(
                    "A manual mapping refers to an invalid field '%s' " % ros_field_selection +
                    "in the ROS message '%s/%s'" %
                    (rule.ros_package_name, rule.ros_message_name),
                    file=sys.stderr)
                continue
            mapping.add_field_pair(mros_selected_fields, ros_selected_fields)
        return mapping

    # apply name based mapping of fields
    mros_field_missing_in_ros = False

    for mros_field in mros_spec.parsed_fields():
        for ros_member in ros_spec.structure.members:
            if mros_field.name.lower() == ros_member.name:
                # get package name and message name from MROS field type
                update_mros_field_information(mros_field, mros_msg.package_name)
                mapping.add_field_pair(mros_field, ros_member)
                break
        else:
            # this allows fields to exist in MROS but not in ROS
            mros_field_missing_in_ros = True

    if mros_field_missing_in_ros:
        # if some fields exist in MROS but not in ROS
        # check that no fields exist in ROS but not in MROS
        # since then it might be the case that those have been renamed and should be mapped
        for ros_member in ros_spec.structure.members:
            for mros_field in mros_spec.parsed_fields():
                if mros_field.name.lower() == ros_member.name:
                    break
            else:
                # if fields from both sides are not mappable the whole message is not mappable
                return None

    return mapping


def load_mros_message(mros_msg):
    msg_context = msgloader.MsgContext.create_default()
    message_path = os.path.join(mros_msg.prefix_path, mros_msg.message_name + '.msg')
    try:
        spec = msgloader.msg_loader.load_msg_from_file(
            msg_context, message_path, '%s/%s' % (mros_msg.package_name, mros_msg.message_name))
        
    except msgloader.InvalidMsgSpec:
        return None
    return spec


def load_mros_service(mros_srv):
    srv_context = msgloader.MsgContext.create_default()
    srv_path = os.path.join(mros_srv.prefix_path, mros_srv.message_name + '.srv')
    srv_name = '%s/%s' % (mros_srv.package_name, mros_srv.message_name)
    try:
        spec = msgloader.msg_loader.load_srv_from_file(srv_context, srv_path, srv_name)
    except msgloader.InvalidMsgSpec:
        return None
    return spec


def load_ros_message(ros_msg):
    message_basepath = os.path.join(ros_msg.prefix_path, 'share')
    message_relative_path = \
        os.path.join(ros_msg.package_name, 'msg', ros_msg.message_name)
    message_path = os.path.join(message_basepath, message_relative_path)
    # Check to see if the message is defined as a .msg file or an .idl file,
    # but preferring '.idl' if both exist.
    if os.path.exists(message_path + '.idl'):
        message_path += '.idl'
        message_relative_path += '.idl'
    elif os.path.exists(message_path + '.msg'):
        message_path += '.msg'
        message_relative_path += '.msg'
    else:
        raise RuntimeError(
            f"message '{ros_msg.package_name}/msg/{ros_msg.message_name}' "
            f"was not found in prefix '{ros_msg.prefix_path}' with either "
            f"file extension '.msg' or '.idl'")
    # We don't support .msg files, but that shouldn't be a problem since an .idl
    # version should have been created when the package was built by rosidl_adapter.
    if message_path.endswith('.msg'):
        raise RuntimeError(
            "mrosbridger cannot process ROS message definitions that lack a '.idl' version, "
            "which normally does not occur as rosidl_adapter should create a '.idl' version "
            "when building the message package which contains the original '.msg' file."
        )
    if not message_path.endswith('.idl'):
        raise RuntimeError(
            f"message_path '{message_path}' unexpectedly does not end with '.idl'"
        )
    idl_locator = \
        rosidl_parser.definition.IdlLocator(message_basepath, message_relative_path)
    spec = rosidl_parser.parser.parse_idl_file(idl_locator)
    messages = spec.content.get_elements_of_type(rosidl_parser.definition.Message)
    if len(messages) != 1:
        raise RuntimeError(
            'unexpectedly found multiple message definitions when processing '
            f"message '{ros_msg.package_name}/msg/{ros_msg.message_name}'"
        )
    return messages[0]


def load_ros_service(ros_srv):
    srv_path = os.path.join(
        ros_srv.prefix_path, 'share', ros_srv.package_name, 'srv',
        ros_srv.message_name + '.srv')
    try:
        spec = rosidl_adapter.parser.parse_service_file(ros_srv.package_name, srv_path)
    except rosidl_adapter.parser.InvalidSpecification:
        return None
    return spec


# make field types hashable
def FieldHash(self):
    return self.name.__hash__()


msgloader.msgs.Field.__hash__ = FieldHash
rosidl_adapter.parser.Field.__hash__ = FieldHash


class Mapping:
    __slots__ = [
        'mros_msg',
        'ros_msg',
        'fields_mros_to_ros',
        'fields_2_to_1',
        'depends_on_ros_messages'
    ]

    def __init__(self, mros_msg, ros_msg):
        self.mros_msg = mros_msg
        self.ros_msg = ros_msg
        self.fields_mros_to_ros = OrderedDict()
        self.fields_2_to_1 = OrderedDict()
        self.depends_on_ros_messages = set()

    def add_field_pair(self, mros_fields, ros_members):
        """
        Add a new mapping for a pair of MROS and ROS messages.

        :type mros_fields: either a msgloader.msgs.Field object with additional attributes `pkg_name`
        and `msg_name` as defined by `update_mros_field_information`, or a tuple of objects of
        that type
        :type ros_members: a single, or list of, rosidl_parser.definition.Member object(s)
        """
        if not isinstance(mros_fields, tuple):
            mros_fields = (mros_fields,)
        if not isinstance(ros_members, tuple):
            ros_members = (ros_members, )
        self.fields_mros_to_ros[mros_fields] = ros_members
        self.fields_2_to_1[ros_members] = mros_fields
        for ros_member in ros_members:
            # If the member is not a namespaced type, skip.
            if not isinstance(ros_member.type, rosidl_parser.definition.NamespacedType):
                continue
            # If it is, then the type will have a namespaced name, e.g. (std_msgs, msg, String)
            # If it is not of the standard ('<package name>', 'msg', '<type>'), skip it
            if len(ros_member.type.namespaces) != 2 or ros_member.type.namespaces[1] != 'msg':
                continue
            # Extract the package name and message name
            pkg_name = ros_member.type.namespaces[0]
            msg_name = ros_member.type.name
            if pkg_name != 'builtin_interfaces':
                self.depends_on_ros_messages.add(Message(pkg_name, msg_name))


def camel_case_to_lower_case_underscore(value):
    # insert an underscore before any upper case letter
    # which is not followed by another upper case letter
    value = re.sub('(.)([A-Z][a-z]+)', '\\1_\\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', '\\1_\\2', value)
    return value.lower()


class MessageIndex:
    """
    Index from package and message names to Message objects.

    Maintains 2 indices from (package_name, message_name) to Message,
    one for MROS messages and another for ROS messages
    """

    def __init__(self):
        self._mros_idx = {}
        self._ros_idx = {}

    def mros_put(self, msg):
        """Add msg to the MROS index."""
        self._mros_idx[(msg.package_name, msg.message_name)] = msg

    def ros_put(self, msg):
        """Add msg to the ROS index."""
        self._ros_idx[(msg.package_name, msg.message_name)] = msg

    def mros_get_from_field(self, field):
        """
        Get Message from MROS index.

        :type field: msgloader.msgs.Field with additional fields `pkg_name`
        and `msg_name` as added by `update_mros_field_information`
        :return: the message indexed for the fields `pkg_name` and
        `msg_name` of `field`
        """
        return self._mros_idx[(field.pkg_name, field.msg_name)]

    def ros_get_from_field(self, field):
        """
        Get Message from ROS index.

        :type field: rosidl_adapter.parser.Field
        :return: the message indexed for the fields `type.pkg_name` and
        `type.type` of `field`
        """
        return self._ros_idx[(field.type.pkg_name, field.type.type)]
