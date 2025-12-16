# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
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

"""Plugin provider for loading python plugins."""

import os
import sys
import traceback
from typing import Dict, List
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from rclpy.node import Node


class PluginDescriptor:
    """Descriptor for a plugin containing its metadata."""

    def __init__(self, plugin_id: str, attributes: Dict[str, str]):
        """
        Initialize the PluginDescriptor.

        Parameters
        ----------
        plugin_id : str
            Unique identifier for the plugin.
        attributes : Dict[str, str]
            Dictionary of plugin attributes.

        """
        self._plugin_id = plugin_id
        self._attributes = attributes

    def plugin_id(self) -> str:
        """
        Get the plugin ID.

        Returns
        -------
        str
            The plugin identifier.

        """
        return self._plugin_id

    def attributes(self) -> Dict[str, str]:
        """
        Get the plugin attributes.

        Returns
        -------
        Dict[str, str]
            Dictionary of plugin attributes.

        """
        return self._attributes


class PluginProvider:
    """
    Provider for discovering and loading python plugins.

    This class is responsible for finding plugin.xml files in ROS2 packages,
    parsing them, and loading the corresponding Python classes.
    """

    def __init__(self, export_tag: str, base_class_type: str):
        """
        Initialize the PluginProvider.

        Parameters
        ----------
        export_tag : str
            The export tag to search for in package.xml files.
        base_class_type : str
            The base class type that plugins must inherit from.

        """
        self._export_tag = export_tag
        self._base_class_type = base_class_type
        self._plugin_descriptors: Dict[str, PluginDescriptor] = {}

    def discover(self, node: Node) -> List[PluginDescriptor]:
        """
        Discover all available plugins.

        Parameters
        ----------
        node : Node
            ROS2 node for logging.

        Returns
        -------
        List[PluginDescriptor]
            List of discovered plugin descriptors.

        """
        plugin_descriptors = []
        plugin_file_list = self._find_plugins(node)

        for package_name, plugin_xml in plugin_file_list:
            plugin_descriptors += self._parse_plugin_xml(package_name, plugin_xml, node)

        # Add discovered plugins to dictionary
        for plugin_descriptor in plugin_descriptors:
            self._plugin_descriptors[plugin_descriptor.plugin_id()] = plugin_descriptor

        return plugin_descriptors

    def load(self, plugin_id: str, node: Node, plugin_context=None):
        """
        Load a plugin by its ID.

        Parameters
        ----------
        plugin_id : str
            The unique identifier of the plugin to load.
        node : Node
            ROS2 node for logging.
        plugin_context : object, optional
            Optional context to pass to the plugin constructor.

        Returns
        -------
        object or None
            An instance of the loaded plugin, or None if loading failed.

        """
        if plugin_id not in self._plugin_descriptors:
            node.get_logger().error(f'PluginProvider.load({plugin_id}): plugin not found')
            return None

        attributes = self._plugin_descriptors[plugin_id].attributes()

        # Add module path to sys.path
        module_path = os.path.join(
            attributes['plugin_path'],
            attributes.get('library_path', '')
        )
        if module_path not in sys.path:
            sys.path.insert(0, module_path)

        try:
            # Import the module
            module = __import__(
                attributes['module_name'],
                fromlist=[attributes['class_from_class_type']],
                level=0
            )
        except Exception:
            node.get_logger().error(
                f'PluginProvider.load({plugin_id}): exception raised in '
                f"__import__({attributes['module_name']}, "
                f"[{attributes['class_from_class_type']}]):\n{traceback.format_exc()}")
            return None

        # Get class reference from module
        class_ref = getattr(module, attributes['class_from_class_type'], None)
        if class_ref is None:
            node.get_logger().error(
                f'PluginProvider.load({plugin_id}): could not find class '
                f'"{attributes["class_from_class_type"]}" in module "{module}"')
            return None

        # Create plugin instance
        try:
            if plugin_context is None:
                return class_ref()
            else:
                return class_ref(plugin_context)
        except Exception:
            node.get_logger().error(
                f'PluginProvider.load({plugin_id}): exception creating instance:\n'
                f'{traceback.format_exc()}')
            return None

    def unload(self, plugin_instance):
        """
        Unload a plugin instance.

        Parameters
        ----------
        plugin_instance : object
            The plugin instance to unload.

        """
        # Cleanup can be added here if needed
        pass

    def _find_plugins(self, node) -> List[tuple]:
        """
        Find all plugin.xml files in ROS2 packages.

        Parameters
        ----------
        node : Node
            ROS2 node for logging.

        Returns
        -------
        List[tuple]
            List of (package_name, plugin_xml_path) tuples.

        """
        plugin_files = []

        # Get all ROS2 packages
        packages = get_packages_with_prefixes()

        for package_name in packages.keys():
            try:
                # Get package share directory
                package_share_dir = get_package_share_directory(package_name)

                # Check for plugin.xml file
                plugin_xml_path = os.path.join(package_share_dir, 'plugin.xml')
                if os.path.isfile(plugin_xml_path):
                    plugin_files.append((package_name, plugin_xml_path))

            except Exception:
                # Package might not be installed or accessible
                continue

        return plugin_files

    def _parse_plugin_xml(
        self,
        package_name: str,
        plugin_xml: str,
        node: Node
    ) -> List[PluginDescriptor]:
        """
        Parse a plugin.xml file and extract plugin descriptors.

        Parameters
        ----------
        package_name : str
            The name of the package containing the plugin.
        plugin_xml : str
            Path to the plugin.xml file.
        node : Node
            ROS2 node for logging.

        Returns
        -------
        List[PluginDescriptor]
            List of plugin descriptors found in the file.

        """
        plugin_descriptors: List[PluginDescriptor] = []

        if not os.path.isfile(plugin_xml):
            node.get_logger().warning(
                f'PluginProvider._parse_plugin_xml() plugin file "{plugin_xml}" '
                f'in package "{package_name}" not found'
            )
            return plugin_descriptors

        try:
            root = ElementTree.parse(plugin_xml)
        except Exception:
            node.get_logger().error(
                f'PluginProvider._parse_plugin_xml() could not parse '
                f'"{plugin_xml}" in package "{package_name}"'
            )
            return plugin_descriptors

        for library_el in root.iter('library'):
            library_path = library_el.attrib.get('path', '')

            for class_el in library_el.iter('class'):
                # Collect common attributes
                attributes = {
                    'package_name': package_name,
                    'plugin_path': os.path.dirname(plugin_xml),
                    'library_path': library_path,
                }

                # Add class attributes
                for key, value in class_el.items():
                    attributes['class_' + key] = value

                # Skip classes with non-matching base_class_type
                class_base_class_type = attributes.get('class_base_class_type', None)
                if class_base_class_type != self._base_class_type:
                    continue

                # Generate unique identifier
                plugin_id = package_name
                if 'class_name' in attributes:
                    plugin_id = plugin_id + '/' + attributes['class_name']
                attributes['plugin_id'] = plugin_id

                # Separate module name and class name from type
                if 'class_type' not in attributes:
                    node.get_logger().warning(
                        f'Plugin in {package_name} missing "type" attribute'
                    )
                    continue

                module_name, class_from_class_type = attributes['class_type'].rsplit('.', 1)
                attributes['module_name'] = module_name
                attributes['class_from_class_type'] = class_from_class_type

                plugin_descriptor = PluginDescriptor(plugin_id, attributes)
                plugin_descriptors.append(plugin_descriptor)

                node.get_logger().info(
                    f'Found plugin: {plugin_id} ({attributes["class_type"]})'
                )

        return plugin_descriptors

    def get_plugin_descriptors(self) -> Dict[str, PluginDescriptor]:
        """
        Get all discovered plugin descriptors.

        Returns
        -------
        Dict[str, PluginDescriptor]
            Dictionary of plugin descriptors indexed by ID.

        """
        return self._plugin_descriptors
