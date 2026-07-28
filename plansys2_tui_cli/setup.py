from setuptools import find_packages, setup

package_name = 'plansys2_tui_cli'

setup(
    name=package_name,
    version='3.1.0',
    packages=find_packages(
        include=[package_name, package_name + '.*'], exclude=['test']
    ),
    include_package_data=True,
    package_data={
        # include the vendored textual library
        'plansys2_tui_cli': ['vendor/**'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description='PlanSys2 TUI (Textual) and ros2cli tools.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plansys2_tui = plansys2_tui_cli.tui.app:run_app',
        ],
        # ros2 plansys2 <verb>
        'ros2cli.command': [
            'plansys2 = plansys2_tui_cli.cli.plansys2_cmd:PlanSys2Command',
        ],
        # Extension point declaration required by ros2cli
        'ros2cli.extension_point': [
            'plansys2.verb = ros2cli.verb:VerbExtension',
        ],
        'plansys2.verb': [
            'knowledge = plansys2_tui_cli.cli.knowledge:KnowledgeVerb',
            'plan_monitor = plansys2_tui_cli.cli.plan_monitor:PlanMonitorVerb',
            'performers = plansys2_tui_cli.cli.performers:PerformersVerb',
            'execution_monitor = plansys2_tui_cli.cli.execution_monitor:ExecutionMonitorVerb',
        ],
    },
)
