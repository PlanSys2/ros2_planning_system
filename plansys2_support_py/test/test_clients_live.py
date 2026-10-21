# Copyright 2026 Intelligent Robotics Lab
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

"""The Python clients against a running PlanSys2, and the Python planner with a plugin."""

import os
import signal
import subprocess
import threading
import time

from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Param
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from plansys2_support_py.core.PluginProvider import PluginDescriptor
from plansys2_support_py.DomainExpertClient import DomainExpertClient
from plansys2_support_py.ExecutorClient import ExecutorClient
from plansys2_support_py.Parser import Parser
from plansys2_support_py.Planner import PlannerNode
from plansys2_support_py.PlannerClient import PlannerClient
from plansys2_support_py.ProblemExpertClient import ProblemExpertClient
import pytest
import python_solvers
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
DOMAIN = os.path.join(TEST_DIR, 'pddl', 'move.pddl')
# A namespace of our own: nothing else running on this host answers
NAMESPACE = f'pytest_{os.getpid()}'


class MovePerformer(ActionExecutorClient):
    """Performs move in two steps, or never ends if endless."""

    def __init__(self, name, endless=False):
        super().__init__(
            name, 0.1, namespace=NAMESPACE,
            parameter_overrides=[Parameter('action_name', value='move')])
        self.endless = endless
        self.progress = 0.0
        self.works = 0

    def do_work(self):
        self.works += 1
        if self.endless:
            self.send_feedback(0.5, 'moving')
            return
        self.progress += 0.5
        if self.progress < 1.0:
            self.send_feedback(self.progress, 'moving')
        else:
            self.progress = 0.0
            self.finish(True, 1.0, 'moved')


class Spinner:
    """Spins some nodes in a thread of their own."""

    def __init__(self, *nodes, executor=None):
        self.executor = executor or SingleThreadedExecutor()
        for node in nodes:
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.executor.shutdown()
        self.thread.join(timeout=5.0)


def wait_until(condition, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


@pytest.fixture(scope='module')
def plansys2():
    """PlanSys2 running in a namespace of its own, and the clients."""
    if not rclpy.ok():
        rclpy.init()
    launch = subprocess.Popen(
        ['ros2', 'launch', 'plansys2_bringup', 'plansys2_bringup_launch_monolithic.py',
         f'model_file:={DOMAIN}', f'namespace:={NAMESPACE}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    # One node for all the clients, which nobody spins: they serve their own nodes
    node = rclpy.create_node('pytest_client', namespace=NAMESPACE)
    clients = {
        'node': node,
        'domain': DomainExpertClient(node),
        'problem': ProblemExpertClient(node),
        'planner': PlannerClient(node),
        'executor': ExecutorClient(node),
    }
    try:
        assert wait_until(lambda: clients['domain'].get_domain_name() == 'move', 60.0)
        assert wait_until(lambda: clients['problem'].get_problem() is not None, 30.0)
        yield clients
    finally:
        for name in ['domain', 'problem', 'planner', 'executor']:
            clients[name].destroy()
        node.destroy_node()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()


def set_problem(problem, robot_room='a', goal='(and (robot_at r1 b))'):
    assert problem.clear_problem_knowledge()
    for name, type_name in [('r1', 'robot'), ('a', 'room'), ('b', 'room')]:
        instance = Param()
        instance.name = name
        instance.type = type_name
        assert problem.add_problem_instance(instance)
    for fact in [f'(robot_at r1 {robot_room})', '(connected a b)', '(connected b a)']:
        assert problem.add_problem_predicate(Parser.node_from_string_predicate(fact))
    assert problem.add_problem_goal(Parser.tree_from_string(goal))


def test_names_resolve_in_the_namespace(plansys2):
    """The namespace applies to every call: this PlanSys2 is only in it."""
    assert plansys2['domain'].get_domain_types() == ['object', 'robot', 'room']
    assert plansys2['domain'].get_domain_actions() == []
    assert plansys2['domain'].get_domain_durative_actions() == ['move']


def test_get_domain_constants(plansys2):
    """The domain expert serves the constants of a type."""
    assert plansys2['domain'].get_domain_constants('room') == ['base']
    assert plansys2['domain'].get_domain_constants('robot') == []
    assert plansys2['domain'].get_domain_constants('spaceship') == []
    assert plansys2['domain'].get_domain_types() == ['object', 'robot', 'room']


def test_get_domain_derived_predicates(plansys2):
    """The derived predicates come with their rules, as from the C++ client."""
    derived = plansys2['domain'].get_domain_derived_predicates()
    assert derived is not None
    assert [d.predicate.name for d in derived] == ['at_base']
    details = plansys2['domain'].get_domain_derived_predicate_details('at_base')
    assert details is not None and len(details) == 1


def test_clients_are_kept(plansys2):
    """A call reuses the client of its service instead of creating one each time."""
    problem = plansys2['problem']
    for _ in range(20):
        assert problem.get_problem_instances() is not None
    names = [c.srv_name for c in problem._node.clients]  # In the node of the client
    assert len([n for n in names if n.endswith('problem_expert/get_problem_instances')]) == 1


def test_plan_and_execute(plansys2):
    """A plan from the planner runs with a Python performer, with feedback and result."""
    set_problem(plansys2['problem'])
    domain = plansys2['domain'].get_domain()
    problem = plansys2['problem'].get_problem()
    plan = plansys2['planner'].get_plan(domain, problem)
    assert plan is not None and len(plan.items) == 1
    assert plan.items[0].action == '(move r1 a b)'

    executor = plansys2['executor']
    assert executor.get_feedback() is None  # It raised before any feedback
    performer = MovePerformer('pytest_mover')
    performer.trigger_configure()
    spinner = Spinner(performer)
    try:
        assert executor.start_plan_execution(plan, verbose=True)
        assert wait_until(lambda: not executor.execute_and_check_plan(verbose=False), 30.0)
        assert executor.get_result().result == ExecutePlan.Result.SUCCESS
        assert executor.get_feedback() is not None
        assert not executor.is_executing_plan()
        facts = [Parser.node_to_string(p) for p in plansys2['problem'].get_problem_predicates()]
        assert '(robot_at r1 b)' in facts
    finally:
        spinner.stop()
        performer.destroy_node()


def test_cancel_plan_execution(plansys2):
    """A running plan is cancelled, and its performer stops."""
    set_problem(plansys2['problem'])
    plan = plansys2['planner'].get_plan(
        plansys2['domain'].get_domain(), plansys2['problem'].get_problem())
    assert plan is not None

    executor = plansys2['executor']
    assert not executor.cancel_plan_execution()  # Nothing to cancel
    performer = MovePerformer('pytest_endless_mover', endless=True)
    performer.trigger_configure()
    spinner = Spinner(performer)
    try:
        assert executor.start_plan_execution(plan)
        assert wait_until(lambda: performer.works > 2, 30.0)
        executor.execute_and_check_plan(verbose=False)
        assert executor.cancel_plan_execution()
        assert not executor.is_executing_plan()
        assert wait_until(lambda: not executor.execute_and_check_plan(verbose=False), 15.0)
        works = performer.works
        time.sleep(0.5)
        assert performer.works == works  # Deactivated: no more work
    finally:
        spinner.stop()
        performer.destroy_node()


def test_each_client_has_a_node_of_its_own(plansys2):
    """Each client has one node, named after the node of the application, and no hidden one."""
    node = plansys2['node']
    expected = {'pytest_client'} | {
        f'pytest_client_{component}_client'
        for component in ['domain_expert', 'problem_expert', 'planner', 'executor']}

    def names():
        return {n for n, ns in node.get_node_names_and_namespaces() if ns == f'/{NAMESPACE}'}

    assert wait_until(lambda: expected <= names(), 10.0)
    assert not [n for n in names() if n.startswith('_')]
    assert {'domain_expert', 'problem_expert', 'planner', 'executor'} <= names()


def test_a_node_nobody_spins_gets_its_answers(plansys2):
    """The application need not spin its node: the client serves its own."""
    node = rclpy.create_node('pytest_unspun_client', namespace=NAMESPACE)
    domain = DomainExpertClient(node)
    try:
        assert domain.get_domain_name() == 'move'
        assert domain.get_domain_name() == 'move'
        # Nor does it change once the node is spun
        spinner = Spinner(node)
        try:
            assert domain.get_domain_name() == 'move'
        finally:
            spinner.stop()
    finally:
        domain.destroy()
        node.destroy_node()


@pytest.mark.parametrize('executor', [SingleThreadedExecutor, MultiThreadedExecutor])
def test_calls_from_a_callback(plansys2, executor):
    """A call from a callback of the node answers, also with the only thread of the executor."""
    problem = plansys2['problem']
    set_problem(problem)
    answers = []
    done = threading.Event()
    ticks = []

    def ask():
        ticks.append(1)
        if not done.is_set():
            answers.append(problem.get_problem_instances())
            answers.append(problem.get_problem_goal())
            done.set()

    node = plansys2['node']
    timer = node.create_timer(0.05, ask)
    spinner = Spinner(node, executor=executor())
    try:
        assert done.wait(15.0)
        assert sorted(i.name for i in answers[0]) == ['a', 'b', 'r1']
        assert answers[1] is not None
        # The executor goes on with the timer
        assert wait_until(lambda: len(ticks) > 3, 5.0)
        # And from another thread while the node is spun
        assert problem.get_problem_goal() is not None
    finally:
        spinner.stop()
        node.destroy_timer(timer)


def test_a_controller_with_its_own_subscription(plansys2):
    """Its subscription and the calls from its timer go on, with the only thread of rclpy.spin."""
    node = rclpy.create_node('pytest_controller', namespace=NAMESPACE)
    problem = ProblemExpertClient(node)
    publisher_node = rclpy.create_node('pytest_controller_publisher', namespace=NAMESPACE)
    publisher = publisher_node.create_publisher(String, 'controller_topic', 10)
    received = []
    answers = []
    node.create_subscription(String, 'controller_topic', received.append, 10)
    node.create_timer(0.05, lambda: answers.append(problem.get_problem_instances() is not None))
    spinner = Spinner(node)
    try:
        assert wait_until(lambda: publisher.get_subscription_count() > 0, 5.0)
        for _ in range(5):
            publisher.publish(String())
            time.sleep(0.05)
        assert wait_until(lambda: len(received) == 5, 5.0)
        assert wait_until(lambda: len(answers) >= 5, 10.0)
        assert all(answers)
    finally:
        spinner.stop()
        problem.destroy()
        publisher_node.destroy_node()
        node.destroy_node()


def test_concurrent_calls_on_a_client(plansys2):
    """Several threads call the same client: one serves its node, all get their answers."""
    problem = plansys2['problem']
    set_problem(problem)
    answers = []
    lock = threading.Lock()

    def caller():
        for _ in range(5):
            instances = problem.get_problem_instances()
            with lock:
                answers.append(instances is not None and len(instances) == 3)

    threads = [threading.Thread(target=caller) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=60.0)
    assert answers == [True] * 20


def python_planner(solvers, timeout):
    """Create a Python planner with FixedSolver plugins, configured and active."""
    namespace = f'{NAMESPACE}_py'
    overrides = [Parameter('plan_solver_plugins', value=list(solvers)),
                 Parameter('plan_solver_timeout', value=timeout)]
    for name, (delay, ignore_cancel) in solvers.items():
        overrides += [Parameter(f'{name}.plugin', value='pytest/fixed'),
                      Parameter(f'{name}.delay', value=delay),
                      Parameter(f'{name}.ignore_cancel', value=ignore_cancel)]
    planner = PlannerNode(namespace=namespace, parameter_overrides=overrides)
    planner.plugin_provider_.get_plugin_descriptors()['pytest/fixed'] = PluginDescriptor(
        'pytest/fixed', {'plugin_path': TEST_DIR, 'module_name': 'python_solvers',
                         'class_from_class_type': 'FixedSolver'})
    planner.trigger_configure()
    planner.trigger_activate()
    client = PlannerClient(rclpy.create_node('pytest_py_planner_client', namespace=namespace))
    return planner, client


@pytest.mark.parametrize('ignore_cancel', [False, True])
def test_python_planner_with_a_plugin(plansys2, ignore_cancel):
    """The Python planner plans with a Python plugin, and a slow one does not hold it."""
    planner, client = python_planner(
        {'quick': (0.0, False), 'slow': (30.0, ignore_cancel)}, 1.0)
    spinner = Spinner(planner, client.node)
    try:
        assert set(planner.solvers_) == {'quick', 'slow'}
        start = time.monotonic()
        plan = client.get_plan('(define (domain move))', '(define (problem p))')
        assert time.monotonic() - start < 3.0
        assert plan is not None and plan.items[0].action == '(move r1 a b)'
        assert client.validate_domain('(define (domain move))')
        assert not client.validate_domain('nothing')
    finally:
        # Every solver ends before the next test, also the one that ignores cancel
        python_solvers.RELEASE.set()
        for _ in python_solvers.STARTED:
            assert python_solvers.RUNNING.acquire(timeout=5.0)
        python_solvers.STARTED.clear()
        python_solvers.RELEASE.clear()
        spinner.stop()
        client.destroy()
        client.node.destroy_node()
        planner.destroy_node()


def test_python_planner_without_plugins_fails_clearly(plansys2):
    """Without Python plugins it does not configure: the C++ ones cannot run in it."""
    planner = PlannerNode(namespace=f'{NAMESPACE}_empty')
    try:
        planner.trigger_configure()
        assert planner._state_machine.current_state[1] == 'unconfigured'
        assert planner.solvers_ == {}
    finally:
        planner.destroy_node()
