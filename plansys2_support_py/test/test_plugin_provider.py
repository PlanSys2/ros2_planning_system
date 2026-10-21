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

"""Unit tests for PluginProvider."""

import os
import tempfile
import unittest
from unittest.mock import patch


from plansys2_support_py.core.PluginProvider import PluginDescriptor, PluginProvider

import rclpy
from rclpy.node import Node


class TestPluginDescriptor(unittest.TestCase):
    """Test suite for PluginDescriptor."""

    def test_initialization(self):
        """Test PluginDescriptor initialization."""
        attributes = {
            'package_name': 'test_package',
            'class_type': 'test_module.TestClass',
            'base_class_type': 'base.BaseClass'
        }
        descriptor = PluginDescriptor('test_plugin', attributes)

        self.assertEqual(descriptor.plugin_id(), 'test_plugin')
        self.assertEqual(descriptor.attributes(), attributes)

    def test_plugin_id(self):
        """Test getting plugin ID."""
        descriptor = PluginDescriptor('my_plugin', {})
        self.assertEqual(descriptor.plugin_id(), 'my_plugin')

    def test_attributes(self):
        """Test getting plugin attributes."""
        attrs = {'key1': 'value1', 'key2': 'value2'}
        descriptor = PluginDescriptor('plugin', attrs)
        self.assertEqual(descriptor.attributes(), attrs)


class TestPluginProvider(unittest.TestCase):
    """Test suite for PluginProvider."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for all tests."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test."""
        self.node = Node('test_node')
        self.provider = PluginProvider(
            export_tag='test_export',
            base_class_type='test.BaseClass'
        )

    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()

    def test_initialization(self):
        """Test PluginProvider initialization."""
        self.assertEqual(self.provider._export_tag, 'test_export')
        self.assertEqual(self.provider._base_class_type, 'test.BaseClass')
        self.assertEqual(self.provider._plugin_descriptors, {})

    def test_parse_plugin_xml_valid(self):
        """Test parsing valid plugin.xml file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as tmp:
            tmp.write("""<?xml version="1.0"?>
<library path="lib">
    <class name="TestPlugin" type="test_module.TestClass" base_class_type="test.BaseClass">
        <description>Test plugin description</description>
    </class>
</library>
""")
            tmp_path = tmp.name

        try:
            descriptors = self.provider._parse_plugin_xml(
                'test_package',
                tmp_path,
                self.node
            )

            self.assertEqual(len(descriptors), 1)
            descriptor = descriptors[0]
            self.assertEqual(descriptor.plugin_id(), 'test_package/TestPlugin')
            attrs = descriptor.attributes()
            self.assertEqual(attrs['package_name'], 'test_package')
            self.assertEqual(attrs['class_type'], 'test_module.TestClass')
            self.assertEqual(attrs['module_name'], 'test_module')
            self.assertEqual(attrs['class_from_class_type'], 'TestClass')
        finally:
            os.remove(tmp_path)

    def test_parse_plugin_xml_wrong_base_class(self):
        """Test parsing plugin.xml with wrong base class."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as tmp:
            tmp.write("""<?xml version="1.0"?>
<library path="lib">
    <class name="TestPlugin" type="test_module.TestClass" base_class_type="wrong.BaseClass">
        <description>Test plugin</description>
    </class>
</library>
""")
            tmp_path = tmp.name

        try:
            descriptors = self.provider._parse_plugin_xml(
                'test_package',
                tmp_path,
                self.node
            )

            # Should skip plugins with non-matching base class
            self.assertEqual(len(descriptors), 0)
        finally:
            os.remove(tmp_path)

    def test_parse_plugin_xml_missing_type(self):
        """Test parsing plugin.xml with missing type attribute."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as tmp:
            tmp.write("""<?xml version="1.0"?>
<library path="lib">
    <class name="TestPlugin" base_class_type="test.BaseClass">
        <description>Test plugin</description>
    </class>
</library>
""")
            tmp_path = tmp.name

        try:
            descriptors = self.provider._parse_plugin_xml(
                'test_package',
                tmp_path,
                self.node
            )

            # Should skip plugins without type attribute
            self.assertEqual(len(descriptors), 0)
        finally:
            os.remove(tmp_path)

    def test_parse_plugin_xml_nonexistent_file(self):
        """Test parsing non-existent plugin.xml file."""
        descriptors = self.provider._parse_plugin_xml(
            'test_package',
            '/nonexistent/plugin.xml',
            self.node
        )

        self.assertEqual(len(descriptors), 0)

    def test_parse_plugin_xml_invalid_xml(self):
        """Test parsing invalid XML file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as tmp:
            tmp.write('<?xml version="1.0"?>\n<invalid><xml>')
            tmp_path = tmp.name

        try:
            descriptors = self.provider._parse_plugin_xml(
                'test_package',
                tmp_path,
                self.node
            )

            self.assertEqual(len(descriptors), 0)
        finally:
            os.remove(tmp_path)

    def test_parse_plugin_xml_multiple_plugins(self):
        """Test parsing plugin.xml with multiple plugins."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as tmp:
            tmp.write("""<?xml version="1.0"?>
<library path="lib">
    <class name="Plugin1" type="module1.Class1" base_class_type="test.BaseClass"/>
    <class name="Plugin2" type="module2.Class2" base_class_type="test.BaseClass"/>
    <class name="Plugin3" type="module3.Class3" base_class_type="wrong.BaseClass"/>
</library>
""")
            tmp_path = tmp.name

        try:
            descriptors = self.provider._parse_plugin_xml(
                'test_package',
                tmp_path,
                self.node
            )

            # Should only get plugins with matching base class
            self.assertEqual(len(descriptors), 2)
            ids = [d.plugin_id() for d in descriptors]
            self.assertIn('test_package/Plugin1', ids)
            self.assertIn('test_package/Plugin2', ids)
        finally:
            os.remove(tmp_path)

#    @patch('plansys2_support_py.core.PluginProvider.get_packages_with_prefixes')
#    @patch('plansys2_support_py.core.PluginProvider.get_package_share_directory')
#    def test_find_plugins(self, mock_get_share_dir, mock_get_packages):
#        """Test finding plugins in packages."""
#        mock_get_packages.return_value = {'pkg1': '/path1', 'pkg2': '/path2'}
#
#        with tempfile.TemporaryDirectory() as tmpdir:
#            # Create plugin.xml in temporary directory
#            plugin_xml_path = os.path.join(tmpdir, 'plugin.xml')
#            with open(plugin_xml_path, 'w') as f:
#                f.write('<?xml version="1.0"?><library/>')
#
#            mock_get_share_dir.return_value = tmpdir
#
#            plugin_files = self.provider._find_plugins(self.node)
#
#            # Should find plugin.xml in both packages
#            self.assertGreaterEqual(len(plugin_files), 1)
#
#    def test_load_nonexistent_plugin(self):
#        """Test loading non-existent plugin."""
#        result = self.provider.load('nonexistent_plugin')
#        self.assertIsNone(result)
#
#    def test_get_plugin_descriptors(self):
#        """Test getting plugin descriptors dictionary."""
#        descriptors = self.provider.get_plugin_descriptors()
#        self.assertIsInstance(descriptors, dict)
#
#    @patch('plansys2_support_py.core.PluginProvider.get_packages_with_prefixes')
#    @patch('plansys2_support_py.core.PluginProvider.get_package_share_directory')
#    def test_discover_plugins(self, mock_get_share_dir, mock_get_packages):
#        """Test discovering plugins."""
#        mock_get_packages.return_value = {'test_pkg': '/test/path'}
#
#        with tempfile.TemporaryDirectory() as tmpdir:
#            # Create valid plugin.xml
#            plugin_xml_path = os.path.join(tmpdir, 'plugin.xml')
#            with open(plugin_xml_path, 'w') as f:
#                f.write("""<?xml version="1.0"?>
# <library path="lib">
#    <class name="TestPlugin" type="test.TestClass" base_class_type="test.BaseClass"/>
# </library>
# """)
#
#            mock_get_share_dir.return_value = tmpdir
#
#            descriptors = self.provider.discover(self.node)
#
#            # Should discover the plugin
#            self.assertGreaterEqual(len(descriptors), 0)
#
#    def test_unload_plugin(self):
#        """Test unloading a plugin."""
#        mock_plugin = MagicMock()
#        # Should not raise any errors
#        self.provider.unload(mock_plugin)
#
#    def test_load_plugin_success(self):
#        """Test successfully loading a plugin."""
#        # Create a test module
#        with tempfile.TemporaryDirectory() as tmpdir:
#            # Create a simple test class
#            test_module_path = os.path.join(tmpdir, 'test_plugin.py')
#            with open(test_module_path, 'w') as f:
#                f.write("""
# class TestClass:
#    def __init__(self):
#        self.initialized = True
# """)
#
#            # Add plugin descriptor
#            attributes = {
#                'plugin_path': tmpdir,
#                'library_path': '',
#                'module_name': 'test_plugin',
#                'class_from_class_type': 'TestClass'
#            }
#            descriptor = PluginDescriptor('test/TestPlugin', attributes)
#            self.provider._plugin_descriptors['test/TestPlugin'] = descriptor
#
#            # Load the plugin
#            result = self.provider.load('test/TestPlugin')
#            self.assertIsNotNone(result)
#            self.assertTrue(hasattr(result, 'initialized'))
#            self.assertTrue(result.initialized)

#     def test_load_plugin_with_context(self):
#         """Test loading a plugin with context."""
#         with tempfile.TemporaryDirectory() as tmpdir:
#             # Create a test class that accepts context
#             test_module_path = os.path.join(tmpdir, 'test_plugin_ctx.py')
#             with open(test_module_path, 'w') as f:
#                 f.write("""
# class TestClassWithContext:
#     def __init__(self, context):
#         self.context = context
# """)
#
#             # Add plugin descriptor
#             attributes = {
#                 'plugin_path': tmpdir,
#                 'library_path': '',
#                 'module_name': 'test_plugin_ctx',
#                 'class_from_class_type': 'TestClassWithContext'
#             }
#             descriptor = PluginDescriptor('test/TestPluginCtx', attributes)
#             self.provider._plugin_descriptors['test/TestPluginCtx'] = descriptor
#
#             # Load the plugin with context
#             context = {'key': 'value'}
#             result = self.provider.load('test/TestPluginCtx', context)
#             self.assertIsNotNone(result)
#             self.assertEqual(result.context, context)
#
#    def test_load_plugin_import_error(self):
#        """Test loading plugin with import error."""
#        attributes = {
#            'plugin_path': '/nonexistent/path',
#            'library_path': '',
#            'module_name': 'nonexistent_module',
#            'class_from_class_type': 'NonexistentClass'
#        }
#        descriptor = PluginDescriptor('test/FailPlugin', attributes)
#        self.provider._plugin_descriptors['test/FailPlugin'] = descriptor
#
#        result = self.provider.load('test/FailPlugin')
#        self.assertIsNone(result)
#
#    def test_load_plugin_class_not_found(self):
#        """Test loading plugin when class is not in module."""
#        with tempfile.TemporaryDirectory() as tmpdir:
#            # Create a module without the expected class
#            test_module_path = os.path.join(tmpdir, 'test_wrong.py')
#            with open(test_module_path, 'w') as f:
#                f.write("""
# class WrongClass:
#    pass
# """)
#
#            # Add plugin descriptor expecting different class
#            attributes = {
#                'plugin_path': tmpdir,
#                'library_path': '',
#                'module_name': 'test_wrong',
#                'class_from_class_type': 'ExpectedClass'
#            }
#            descriptor = PluginDescriptor('test/WrongPlugin', attributes)
#            self.provider._plugin_descriptors['test/WrongPlugin'] = descriptor
#
#            result = self.provider.load('test/WrongPlugin')
#            self.assertIsNone(result)
#
#    def test_load_plugin_instantiation_error(self):
#        """Test loading plugin with instantiation error."""
#        with tempfile.TemporaryDirectory() as tmpdir:
#            # Create a class that raises error on initialization
#            test_module_path = os.path.join(tmpdir, 'test_error.py')
#            with open(test_module_path, 'w') as f:
#                f.write("""
# class ErrorClass:
#    def __init__(self):
#        raise RuntimeError('Initialization error')
# """)
#
#            # Add plugin descriptor
#            attributes = {
#                'plugin_path': tmpdir,
#                'library_path': '',
#                'module_name': 'test_error',
#                'class_from_class_type': 'ErrorClass'
#            }
#            descriptor = PluginDescriptor('test/ErrorPlugin', attributes)
#            self.provider._plugin_descriptors['test/ErrorPlugin'] = descriptor
#
#            result = self.provider.load('test/ErrorPlugin')
#            self.assertIsNone(result)
#
#    def test_load_plugin_with_library_path(self):
#        """Test loading plugin with library_path set."""
#        with tempfile.TemporaryDirectory() as tmpdir:
#            # Create subdirectory for library
#            lib_dir = os.path.join(tmpdir, 'lib')
#            os.makedirs(lib_dir)
#
#            # Create test module in lib directory
#            test_module_path = os.path.join(lib_dir, 'test_lib.py')
#            with open(test_module_path, 'w') as f:
#                f.write("""
# class LibClass:
#    def __init__(self):
#        self.from_lib = True
# """)
#
#            # Add plugin descriptor with library_path
#            attributes = {
#                'plugin_path': tmpdir,
#                'library_path': 'lib',
#                'module_name': 'test_lib',
#                'class_from_class_type': 'LibClass'
#            }
#            descriptor = PluginDescriptor('test/LibPlugin', attributes)
#            self.provider._plugin_descriptors['test/LibPlugin'] = descriptor
#
#            result = self.provider.load('test/LibPlugin')
#            self.assertIsNotNone(result)
#            self.assertTrue(result.from_lib)

    @patch('plansys2_support_py.core.PluginProvider.get_packages_with_prefixes')
    def test_find_plugins_with_exception(self, mock_get_packages):
        """Test _find_plugins handles exceptions gracefully."""
        # Mock packages that will cause exceptions
        mock_get_packages.return_value = {
            'good_pkg': '/path/to/good',
            'bad_pkg': '/path/to/bad'
        }

        # Mock get_package_share_directory to raise for bad_pkg
        with patch(
            'plansys2_support_py.core.PluginProvider.get_package_share_directory'
        ) as mock_share:
            def side_effect(pkg_name):
                if pkg_name == 'bad_pkg':
                    raise Exception('Package not found')
                return '/tmp/test'

            mock_share.side_effect = side_effect

            # Should not raise, just skip bad packages
            plugin_files = self.provider._find_plugins(self.node)
            # Result should not be None (graceful handling)
            self.assertIsInstance(plugin_files, list)


if __name__ == '__main__':
    unittest.main()
