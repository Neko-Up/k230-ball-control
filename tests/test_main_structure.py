import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "main.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
PURE_CONSTANTS = {
    target.id: ast.literal_eval(item.value)
    for item in TREE.body
    if isinstance(item, ast.Assign)
    for target in item.targets
    if isinstance(target, ast.Name)
    and target.id in {
        "MIN_BOX_SIZE", "MAX_BOX_SIZE", "MAX_ASPECT_RATIO",
        "BLOB_THRESHOLDS", "BLOB_GLOBAL_ROI", "BLOB_ROI_HALF_WIDTH",
        "BLOB_MIN_PIXELS", "BLOB_MAX_PIXELS", "BLOB_MAX_ASPECT_RATIO",
        "BLOB_MAX_CENTER_DISTANCE",
        "TRACK_SEARCH", "TRACK_ACTIVE", "TRACK_RECOVER",
        "AI_VALIDATE_INTERVAL", "BLOB_LOST_TO_RECOVER",
        "AI_FAILURES_TO_RECOVER", "AI_BLOB_IDENTITY_MAX_DISTANCE",
        "METRICS_EVERY_N_CONTROL_FRAMES",
    }
}


def load_pure_function(name, namespace=None):
    node = next(
        (
            item
            for item in TREE.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        ),
        None,
    )
    assert node is not None, "{} is missing".format(name)
    module = ast.Module(body=[node], type_ignores=[])
    injected_namespace = dict(PURE_CONSTANTS)
    if namespace is not None:
        injected_namespace.update(namespace)
    exec(compile(module, str(SOURCE), "exec"), injected_namespace)
    return injected_namespace[name]


CONTROL_BOUNDARY_NAMES = {
    "publish_measurement", "update_servo_control",
    "publish_control_outputs", "draw_osd", "format_deviation_msg",
    "invalidate_control_state",
}
UART_OBJECT_NAMES = {"uart", "uart_obj"}


def rtsp_worker_control_calls(tree, worker_class_name, worker_method_name):
    module_functions = {
        item.name: item
        for item in tree.body
        if isinstance(item, ast.FunctionDef)
    }
    class_methods = {
        item.name: {
            method.name: method
            for method in item.body
            if isinstance(method, ast.FunctionDef)
        }
        for item in tree.body
        if isinstance(item, ast.ClassDef)
    }
    node_by_key = {
        ("module", name): node
        for name, node in module_functions.items()
    }
    node_by_key.update({
        ("class", class_name, method_name): node
        for class_name, methods in class_methods.items()
        for method_name, node in methods.items()
    })

    uart_value = ("uart-value",)
    worker_instance_value = ("worker-instance",)
    foreign_instance_value = ("foreign-instance",)
    local_class_names = {}
    local_defining_scopes = {}
    environment_scopes = {}
    control_calls = set()

    def copy_bindings(bindings):
        return {
            name: set(values)
            for name, values in bindings.items()
        }

    def merge_bindings(bindings_list):
        merged = {}
        for bindings in bindings_list:
            for name, values in bindings.items():
                merged.setdefault(name, set()).update(values)
        return merged

    def replace_bindings(destination, source):
        destination.clear()
        destination.update(copy_bindings(source))

    def new_environment_scope(environment, instance_state):
        scope = {
            "environment": environment,
            "instance_state": instance_state,
        }
        environment_scopes[id(environment)] = scope
        return scope

    def copy_environment(environment):
        copied = copy_bindings(environment)
        scope = environment_scopes.get(id(environment))
        if scope is not None:
            environment_scopes[id(copied)] = scope
        return copied

    def positional_parameter_names(node):
        return [
            argument.arg
            for argument in node.args.posonlyargs + node.args.args
        ]

    def all_parameter_names(node):
        names = positional_parameter_names(node)
        names.extend(argument.arg for argument in node.args.kwonlyargs)
        if node.args.vararg is not None:
            names.append(node.args.vararg.arg)
        if node.args.kwarg is not None:
            names.append(node.args.kwarg.arg)
        return names

    def expression_values(expression, environment, instance_state,
                          class_name):
        if isinstance(expression, ast.Name):
            if expression.id in environment:
                return set(environment[expression.id])
            if expression.id in CONTROL_BOUNDARY_NAMES:
                return {("boundary", expression.id)}
            if expression.id in UART_OBJECT_NAMES:
                return {uart_value}
            return set()
        if not isinstance(expression, ast.Attribute):
            return set()
        if (isinstance(expression.value, ast.Name)
                and expression.value.id == "self"):
            if expression.attr in instance_state:
                return set(instance_state[expression.attr])
            if (expression.attr in UART_OBJECT_NAMES
                    and worker_instance_value in environment.get(
                        "self", set())):
                return {uart_value}
            if expression.attr in class_methods.get(class_name, {}):
                return {
                    (
                        "bound-method", class_name, expression.attr,
                        worker_instance_value in environment.get(
                            "self", set()),
                    )
                }
            return set()

        values = set()
        for owner in expression_values(
                expression.value, environment, instance_state, class_name):
            if (owner[0] == "class"
                    and expression.attr in class_methods.get(owner[1], {})):
                values.add(
                    ("unbound-method", owner[1], expression.attr))
        return values

    def bind_assignment_target(target, values, environment,
                               instance_state):
        if isinstance(target, ast.Name):
            environment[target.id] = set(values)
            return
        if (isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"):
            instance_state[target.attr] = set(values)
            return
        if isinstance(target, (ast.List, ast.Tuple)):
            for element in target.elts:
                bind_assignment_target(
                    element, set(), environment, instance_state)
        elif isinstance(target, ast.Starred):
            bind_assignment_target(
                target.value, set(), environment, instance_state)

    def is_uart_constructor(expression):
        if not isinstance(expression, ast.Call):
            return False
        return (
            isinstance(expression.func, ast.Name)
            and expression.func.id == "UART"
        ) or (
            isinstance(expression.func, ast.Attribute)
            and expression.func.attr == "UART"
        )

    def register_local_function(node, environment, class_name):
        key = ("local", id(node))
        node_by_key[key] = node
        local_class_names[key] = class_name
        local_defining_scopes[key] = environment_scopes[id(environment)]
        environment[node.name] = {key}

    def analyze_expression(expression, environment, instance_state,
                           class_name, active):
        if expression is None or isinstance(expression, ast.Lambda):
            return
        if isinstance(expression, ast.Call):
            analyze_expression(
                expression.func, environment, instance_state,
                class_name, active)
            targets = expression_values(
                expression.func, environment, instance_state, class_name)
            write_is_uart = (
                isinstance(expression.func, ast.Attribute)
                and expression.func.attr == "write"
                and uart_value in expression_values(
                    expression.func.value, environment,
                    instance_state, class_name)
            )

            positional_values = []
            for argument in expression.args:
                analyze_expression(
                    argument, environment, instance_state,
                    class_name, active)
                positional_values.append(expression_values(
                    argument, environment, instance_state, class_name))

            keyword_values = []
            for keyword in expression.keywords:
                analyze_expression(
                    keyword.value, environment, instance_state,
                    class_name, active)
                keyword_values.append((
                    keyword.arg,
                    expression_values(
                        keyword.value, environment,
                        instance_state, class_name),
                ))

            process_call(
                targets, positional_values, keyword_values,
                write_is_uart, environment, instance_state,
                class_name, active)
            return
        if isinstance(expression, ast.NamedExpr):
            analyze_expression(
                expression.value, environment, instance_state,
                class_name, active)
            bind_assignment_target(
                expression.target,
                expression_values(
                    expression.value, environment,
                    instance_state, class_name),
                environment, instance_state)
            return
        for child in ast.iter_child_nodes(expression):
            if isinstance(child, ast.expr):
                analyze_expression(
                    child, environment, instance_state,
                    class_name, active)

    def analyze_branches(branches, environment, instance_state,
                         class_name, active):
        environment_results = []
        instance_results = []
        for statements, initial_environment, initial_instance in branches:
            branch_environment = copy_environment(initial_environment)
            branch_instance = copy_bindings(initial_instance)
            analyze_statements(
                statements, branch_environment, branch_instance,
                class_name, active)
            environment_results.append(branch_environment)
            instance_results.append(branch_instance)
        replace_bindings(
            environment, merge_bindings(environment_results))
        replace_bindings(
            instance_state, merge_bindings(instance_results))

    def analyze_statements(statements, environment, instance_state,
                           class_name, active):
        for statement in statements:
            if isinstance(statement, ast.FunctionDef):
                for decorator in statement.decorator_list:
                    analyze_expression(
                        decorator, environment, instance_state,
                        class_name, active)
                for default in (
                        list(statement.args.defaults)
                        + [value for value in statement.args.kw_defaults
                           if value is not None]):
                    analyze_expression(
                        default, environment, instance_state,
                        class_name, active)
                register_local_function(
                    statement, environment, class_name)
                continue
            if isinstance(statement, (ast.ClassDef, ast.AsyncFunctionDef)):
                continue
            if isinstance(statement, ast.Assign):
                analyze_expression(
                    statement.value, environment, instance_state,
                    class_name, active)
                values = expression_values(
                    statement.value, environment,
                    instance_state, class_name)
                for target in statement.targets:
                    bind_assignment_target(
                        target, values, environment, instance_state)
                continue
            if isinstance(statement, ast.AnnAssign):
                analyze_expression(
                    statement.value, environment, instance_state,
                    class_name, active)
                bind_assignment_target(
                    statement.target,
                    expression_values(
                        statement.value, environment,
                        instance_state, class_name),
                    environment, instance_state)
                continue
            if isinstance(statement, ast.AugAssign):
                analyze_expression(
                    statement.target, environment, instance_state,
                    class_name, active)
                analyze_expression(
                    statement.value, environment, instance_state,
                    class_name, active)
                bind_assignment_target(
                    statement.target, set(),
                    environment, instance_state)
                continue
            if isinstance(statement, ast.If):
                analyze_expression(
                    statement.test, environment, instance_state,
                    class_name, active)
                base_environment = copy_environment(environment)
                base_instance = copy_bindings(instance_state)
                analyze_branches(
                    [
                        (statement.body, base_environment, base_instance),
                        (statement.orelse, base_environment, base_instance),
                    ],
                    environment, instance_state, class_name, active)
                continue
            if isinstance(statement, (ast.For, ast.AsyncFor)):
                analyze_expression(
                    statement.iter, environment, instance_state,
                    class_name, active)
                base_environment = copy_environment(environment)
                base_instance = copy_bindings(instance_state)
                body_environment = copy_environment(base_environment)
                body_instance = copy_bindings(base_instance)
                bind_assignment_target(
                    statement.target, set(),
                    body_environment, body_instance)
                analyze_statements(
                    statement.body, body_environment, body_instance,
                    class_name, active)
                replace_bindings(
                    environment,
                    merge_bindings([base_environment, body_environment]))
                replace_bindings(
                    instance_state,
                    merge_bindings([base_instance, body_instance]))
                analyze_statements(
                    statement.orelse, environment, instance_state,
                    class_name, active)
                continue
            if isinstance(statement, ast.While):
                analyze_expression(
                    statement.test, environment, instance_state,
                    class_name, active)
                base_environment = copy_environment(environment)
                base_instance = copy_bindings(instance_state)
                body_environment = copy_environment(base_environment)
                body_instance = copy_bindings(base_instance)
                analyze_statements(
                    statement.body, body_environment, body_instance,
                    class_name, active)
                replace_bindings(
                    environment,
                    merge_bindings([base_environment, body_environment]))
                replace_bindings(
                    instance_state,
                    merge_bindings([base_instance, body_instance]))
                analyze_statements(
                    statement.orelse, environment, instance_state,
                    class_name, active)
                continue
            if isinstance(statement, (ast.Try, ast.TryStar)):
                base_environment = copy_environment(environment)
                base_instance = copy_bindings(instance_state)

                normal_environment = copy_environment(base_environment)
                normal_instance = copy_bindings(base_instance)
                analyze_statements(
                    statement.body, normal_environment, normal_instance,
                    class_name, active)
                analyze_statements(
                    statement.orelse, normal_environment, normal_instance,
                    class_name, active)

                branch_environments = [normal_environment]
                branch_instances = [normal_instance]
                for handler in statement.handlers:
                    handler_environment = copy_environment(base_environment)
                    handler_instance = copy_bindings(base_instance)
                    analyze_expression(
                        handler.type, handler_environment,
                        handler_instance, class_name, active)
                    if handler.name is not None:
                        handler_environment[handler.name] = set()
                    analyze_statements(
                        handler.body, handler_environment,
                        handler_instance, class_name, active)
                    branch_environments.append(handler_environment)
                    branch_instances.append(handler_instance)

                replace_bindings(
                    environment, merge_bindings(branch_environments))
                replace_bindings(
                    instance_state, merge_bindings(branch_instances))
                analyze_statements(
                    statement.finalbody, environment, instance_state,
                    class_name, active)
                continue
            if isinstance(statement, (ast.With, ast.AsyncWith)):
                for item in statement.items:
                    analyze_expression(
                        item.context_expr, environment, instance_state,
                        class_name, active)
                    if item.optional_vars is not None:
                        bind_assignment_target(
                            item.optional_vars, set(),
                            environment, instance_state)
                analyze_statements(
                    statement.body, environment, instance_state,
                    class_name, active)
                continue
            if isinstance(statement, ast.Match):
                analyze_expression(
                    statement.subject, environment, instance_state,
                    class_name, active)
                base_environment = copy_environment(environment)
                base_instance = copy_bindings(instance_state)
                branches = [
                    (case.body, base_environment, base_instance)
                    for case in statement.cases
                ]
                branches.append(
                    ([], base_environment, base_instance))
                analyze_branches(
                    branches, environment, instance_state,
                    class_name, active)
                continue

            for child in ast.iter_child_nodes(statement):
                if isinstance(child, ast.expr):
                    analyze_expression(
                        child, environment, instance_state,
                        class_name, active)
                elif isinstance(child, ast.stmt):
                    analyze_statements(
                        [child], environment, instance_state,
                        class_name, active)

    def analyze_callable(target, positional_values, keyword_values,
                         caller_environment, caller_instance, active):
        kind = target[0]
        if kind == "module":
            node_key = target
            class_name = None
            bound_method = False
            uses_worker_instance = False
            uses_current_instance = False
            base_environment = module_environment
            source_instance = caller_instance
        elif kind == "local":
            node_key = target
            class_name = local_class_names.get(target)
            bound_method = False
            defining_scope = local_defining_scopes[target]
            caller_scope = environment_scopes[id(caller_environment)]
            same_defining_scope = caller_scope is defining_scope
            base_environment = (
                caller_environment
                if same_defining_scope
                else defining_scope["environment"]
            )
            source_instance = (
                caller_instance
                if same_defining_scope
                else defining_scope["instance_state"]
            )
            uses_worker_instance = (
                worker_instance_value in base_environment.get(
                    "self", set())
            )
            uses_current_instance = (
                uses_worker_instance
                or foreign_instance_value in base_environment.get(
                    "self", set())
            )
        elif kind in {"bound-method", "unbound-method"}:
            node_key = ("class", target[1], target[2])
            class_name = target[1]
            bound_method = kind == "bound-method"
            receiver_values = (
                set() if bound_method or not positional_values
                else positional_values[0]
            )
            uses_worker_instance = (
                target[3] if bound_method
                else worker_instance_value in receiver_values
            )
            uses_current_instance = (
                bound_method
                or uses_worker_instance
                or foreign_instance_value in receiver_values
            )
            base_environment = module_environment
            source_instance = caller_instance
        else:
            return copy_bindings(caller_instance)

        if node_key in active:
            return copy_bindings(caller_instance)

        node = node_by_key[node_key]
        callee_environment = copy_bindings(base_environment)
        for name in all_parameter_names(node):
            callee_environment[name] = set()

        positional_names = positional_parameter_names(node)
        if bound_method and positional_names:
            positional_names = positional_names[1:]
        for index, values in enumerate(positional_values):
            if index < len(positional_names):
                callee_environment[positional_names[index]] = set(values)

        ignored_bound_name = None
        full_positional_names = positional_parameter_names(node)
        if full_positional_names and kind in {
                "bound-method", "unbound-method"}:
            receiver_name = full_positional_names[0]
            if bound_method:
                ignored_bound_name = receiver_name
            if uses_worker_instance:
                callee_environment[receiver_name] = {
                    worker_instance_value,
                }
            else:
                callee_environment[receiver_name] = {
                    foreign_instance_value,
                }
        for name, values in keyword_values:
            if (name is not None
                    and name in callee_environment
                    and name != ignored_bound_name):
                callee_environment[name] = set(values)

        callee_instance = (
            copy_bindings(source_instance)
            if uses_current_instance else {}
        )
        new_environment_scope(callee_environment, callee_instance)
        active.add(node_key)
        try:
            analyze_statements(
                node.body, callee_environment, callee_instance,
                class_name, active)
        finally:
            active.remove(node_key)
        if uses_current_instance:
            return callee_instance
        return copy_bindings(caller_instance)

    def process_call(targets, positional_values, keyword_values,
                     write_is_uart, environment, instance_state,
                     class_name, active):
        if write_is_uart:
            control_calls.add("uart.write")

        resulting_instances = []
        for target in targets:
            if target[0] == "boundary":
                control_calls.add(target[1])
                resulting_instances.append(
                    copy_bindings(instance_state))
            elif target[0] in {
                    "module", "local",
                    "bound-method", "unbound-method"}:
                resulting_instances.append(analyze_callable(
                    target, positional_values, keyword_values,
                    environment, instance_state, active))
            else:
                resulting_instances.append(
                    copy_bindings(instance_state))
        if resulting_instances:
            replace_bindings(
                instance_state,
                merge_bindings(resulting_instances))

    module_environment = {}
    module_instance_state = {}
    new_environment_scope(module_environment, module_instance_state)
    for statement in tree.body:
        if isinstance(statement, ast.FunctionDef):
            if statement.name in CONTROL_BOUNDARY_NAMES:
                module_environment[statement.name] = {
                    ("boundary", statement.name)
                }
            else:
                module_environment[statement.name] = {
                    ("module", statement.name)
                }
        elif isinstance(statement, ast.ClassDef):
            module_environment[statement.name] = {
                ("class", statement.name)
            }
        elif isinstance(statement, ast.Assign):
            values = expression_values(
                statement.value, module_environment,
                module_instance_state, None)
            for target in statement.targets:
                target_values = values
                if (isinstance(target, ast.Name)
                        and target.id in UART_OBJECT_NAMES
                        and is_uart_constructor(statement.value)):
                    target_values = {uart_value}
                bind_assignment_target(
                    target, target_values, module_environment,
                    module_instance_state)
        elif isinstance(statement, ast.AnnAssign):
            values = expression_values(
                statement.value, module_environment,
                module_instance_state, None)
            if (isinstance(statement.target, ast.Name)
                    and statement.target.id in UART_OBJECT_NAMES
                    and is_uart_constructor(statement.value)):
                values = {uart_value}
            bind_assignment_target(
                statement.target, values,
                module_environment, module_instance_state)

    root_key = ("class", worker_class_name, worker_method_name)
    root = node_by_key[root_key]
    root_environment = copy_bindings(module_environment)
    for name in all_parameter_names(root):
        root_environment[name] = (
            {uart_value} if name in UART_OBJECT_NAMES else set()
        )
    root_instance_state = {
        name: {uart_value}
        for name in UART_OBJECT_NAMES
    }
    new_environment_scope(root_environment, root_instance_state)
    root_positional_names = positional_parameter_names(root)
    if root_positional_names:
        root_environment[root_positional_names[0]] = {
            worker_instance_value,
        }
    analyze_statements(
        root.body, root_environment, root_instance_state,
        worker_class_name, {root_key})
    return control_calls
def test_detection_circle_geometry():
    detection_circle = load_pure_function("detection_circle")
    assert detection_circle(10, 20, 20, 12) == (20, 26, 13)
    assert detection_circle(1, 2, 2, 2) == (2, 3, 4)


def test_single_ai_capture_selects_highest_valid_confidence():
    select_best_ai_ball = load_pure_function("select_best_ai_ball")
    detections = [
        [0, 0.40, 10, 10, 30, 30],
        [0, 0.90, 100, 100, 130, 130],
        [0, 0.99, 0, 0, 300, 10],
    ]
    result = select_best_ai_ball(detections)
    assert result["cx"] == 115
    assert result["cy"] == 115
    assert result["score"] == 0.90


def test_single_ai_capture_rejects_invalid_configured_boxes():
    select_best_ai_ball = load_pure_function(
        "select_best_ai_ball",
        {"MIN_BOX_SIZE": 20, "MAX_BOX_SIZE": 170, "MAX_ASPECT_RATIO": 1.8},
    )
    detections = [
        [0, 0.99, 0, 0, 10, 10],
        [0, 0.80, 0, 0, 40, 20],
    ]
    assert select_best_ai_ball(detections) is None


def test_blob_candidate_prefers_nearest_valid_ball():
    select_blob_candidate = load_pure_function("select_blob_candidate")
    candidates = [
        {"x": 90, "y": 180, "w": 20, "h": 18, "pixels": 240},
        {"x": 200, "y": 180, "w": 22, "h": 20, "pixels": 300},
        {"x": 105, "y": 180, "w": 80, "h": 5, "pixels": 300},
    ]
    result = select_blob_candidate(candidates, 100, 190)
    assert result["x"] == 90


def test_blob_detection_returns_plain_candidate_from_requested_roi():
    select_blob_candidate = load_pure_function("select_blob_candidate")
    detect_blob_measurement = load_pure_function(
        "detect_blob_measurement",
        {"select_blob_candidate": select_blob_candidate},
    )

    class Blob:
        def rect(self):
            return (90, 180, 20, 18)

        def pixels(self):
            return 240

    class Image:
        def __init__(self):
            self.call = None

        def find_blobs(self, thresholds, **kwargs):
            self.call = (thresholds, kwargs)
            return [Blob()]

    img = Image()
    roi = (20, 110, 192, 140)
    result = detect_blob_measurement(img, roi)

    assert result == {"x": 90, "y": 180, "w": 20, "h": 18, "pixels": 240}
    assert img.call == (
        [(0, 70, -20, 20, -20, 20)],
        {
            "roi": roi,
            "pixels_threshold": 40,
            "area_threshold": 40,
            "merge": False,
        },
    )


def test_blob_detection_uses_predicted_center_when_roi_is_edge_clamped():
    select_blob_candidate = load_pure_function("select_blob_candidate")
    detect_blob_measurement = load_pure_function(
        "detect_blob_measurement",
        {"select_blob_candidate": select_blob_candidate},
    )

    class Blob:
        def rect(self):
            return (620, 180, 20, 18)

        def pixels(self):
            return 240

    class Image:
        def find_blobs(self, thresholds, **kwargs):
            return [Blob()]

    result = detect_blob_measurement(
        Image(), (448, 110, 192, 140), 639, 190)
    assert result["x"] == 620


def test_blob_tracking_roi_stays_inside_the_global_rod_region():
    blob_tracking_roi = load_pure_function("blob_tracking_roi")
    assert blob_tracking_roi(100) == (4, 110, 192, 140)
    assert blob_tracking_roi(0) == (0, 110, 192, 140)
    assert blob_tracking_roi(639) == (448, 110, 192, 140)


def test_hybrid_tracking_transitions():
    transition = load_pure_function("hybrid_transition")
    assert transition("SEARCH", False, True, 0, 0) == "TRACK"
    assert transition("TRACK", False, False, 1, 0) == "TRACK"
    assert transition("TRACK", False, False, 2, 0) == "RECOVER"
    assert transition("TRACK", True, False, 0, 2) == "RECOVER"
    assert transition("RECOVER", False, True, 0, 0) == "TRACK"


def test_kpu_reacquisition_clears_blob_misses_before_returning_to_track():
    transition = load_pure_function("hybrid_transition")
    ai_capture_counters = load_pure_function("ai_capture_counters")

    state = transition("TRACK", False, False, 2, 0)
    assert state == "RECOVER"
    blob_misses, ai_failures, predicted_frames = ai_capture_counters(
        state, False, 2)
    state = transition(
        state, False, True, blob_misses, ai_failures)

    assert state == "TRACK"
    assert (blob_misses, ai_failures, predicted_frames) == (0, 0, 0)
    assert transition("TRACK", False, False, 1, 0) == "TRACK"


def test_kpu_capture_within_blob_identity_gate_is_valid():
    identity_match = load_pure_function("kpu_blob_identity_match")
    capture = {"cx": 130, "cy": 115}
    assert identity_match(100, 100, capture) is True


def test_distant_kpu_capture_fails_without_overwriting_blob_control():
    identity_match = load_pure_function("kpu_blob_identity_match")
    transition = load_pure_function("hybrid_transition")
    validation_outcome = load_pure_function(
        "kpu_validation_outcome",
        {"kpu_blob_identity_match": identity_match},
    )
    published_control = {
        "x": 116, "y": 101, "vx": 0.2, "vy": 0.0,
        "valid": True, "source": "blob", "timestamp_ms": 50,
    }
    before = dict(published_control)

    returned, valid, failures = validation_outcome(
        published_control, 100, 100, {"cx": 300, "cy": 100}, 1)

    assert returned is published_control
    assert returned == before
    assert valid is False
    assert failures == 2
    assert transition("TRACK", True, valid, 0, failures) == "RECOVER"


def test_hybrid_schedule_is_control_first_with_six_frame_validation():
    schedule = load_pure_function("hybrid_frame_actions")
    assert PURE_CONSTANTS["AI_VALIDATE_INTERVAL"] == 6
    assert schedule("TRACK", 2, True) == ("blob_control",)
    assert schedule("TRACK", 3, True) == ("blob_control",)
    assert schedule("TRACK", 5, True) == ("blob_control",)
    assert schedule("TRACK", 6, True) == ("blob_control", "kpu")
    assert schedule("TRACK", 12, True) == ("blob_control", "kpu")
    assert schedule("SEARCH", 5, True) == ("kpu",)
    assert schedule("RECOVER", 5, True) == ("kpu",)
    assert schedule("TRACK", 5, False) == ("kpu",)


def test_one_missed_blob_frame_is_predicted_then_requests_invalidation():
    predict_position = load_pure_function("predict_position")
    predict_miss = load_pure_function(
        "prediction_for_missed_frame",
        {
            "predict_position": predict_position,
            "PREDICTION_HORIZON_MS": 35,
            "PREDICTION_MAX_SHIFT_PX": 16,
            "PREDICT_ONLY_MAX_FRAMES": 1,
        },
    )
    control = {
        "x": 100, "y": 50, "vx": 0.2, "vy": 0.1,
        "valid": True, "source": "blob", "timestamp_ms": 0,
    }

    predicted, predicted_frames = predict_miss(control, 0, 40)
    assert predicted == {
        "x": 107, "y": 54, "vx": 0.2, "vy": 0.1,
        "valid": True, "source": "predict", "timestamp_ms": 40,
    }
    assert predicted_frames == 1
    assert predict_miss(predicted, predicted_frames, 80) == (None, 1)


def test_tracking_marker_uses_state_color_and_skips_invalid_control():
    tracking_osd_color = load_pure_function("tracking_osd_color")
    draw_tracking_marker = load_pure_function(
        "draw_tracking_marker",
        {
            "tracking_osd_color": tracking_osd_color,
            "ai_to_disp": lambda x, y: (x, y),
        },
    )

    class Image:
        def __init__(self):
            self.circles = []

        def draw_circle(self, x, y, radius, **kwargs):
            self.circles.append((x, y, radius, kwargs))

    img = Image()
    valid = {"x": 120, "y": 80, "valid": True}
    draw_tracking_marker(img, valid, "SEARCH")
    draw_tracking_marker(img, valid, "TRACK")
    draw_tracking_marker(img, valid, "RECOVER")
    draw_tracking_marker(img, {"x": 0, "y": 0, "valid": False}, "TRACK")

    assert [circle[3]["color"] for circle in img.circles] == [
        (0, 255, 255, 255),
        (0, 255, 0, 255),
        (0, 0, 255, 255),
    ]
    assert all(circle[:3] == (120, 80, 12) for circle in img.circles)


def test_control_outputs_call_servo_hook_before_uart_osd_display():
    events = []

    class OSD:
        def clear(self):
            events.append("clear")

    class Display:
        LAYER_OSD3 = 3

        @staticmethod
        def show_image(*args):
            events.append("display")

    shared_state = {"valid": True, "x": 10, "y": 20}
    publish_outputs = load_pure_function(
        "publish_control_outputs",
        {
            "frame_counter": 5,
            "OSD_EVERY_N_FRAMES": 1,
            "control_state": shared_state,
            "should_render_osd": lambda frame, cadence: True,
            "update_servo_control": lambda state: events.append(
                ("servo", state)),
            "draw_osd": lambda *args: events.append("uart_osd"),
            "Display": Display,
        },
    )

    publish_outputs(OSD(), None, [], object())
    assert events == [
        "clear",
        ("servo", shared_state),
        "uart_osd",
        "display",
    ]


def test_kpu_frame_resources_are_released_before_blob_only_frames():
    detection = next(
        item
        for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == "detection"
    )
    required = {
        "rgb888p_img", "ai2d_input", "ai2d_input_tensor", "results",
    }
    released_together = False
    for node in ast.walk(detection):
        if not isinstance(node, ast.Try) or not node.finalbody:
            continue
        released = {
            target.id
            for statement in node.finalbody
            if isinstance(statement, ast.Delete)
            for target in statement.targets
            if isinstance(target, ast.Name)
        }
        if required <= released:
            released_together = True
            break
    assert released_together


def test_validation_loop_commits_control_before_kpu_inference():
    detection = next(
        item
        for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == "detection"
    )
    action_loop = next(
        node
        for node in ast.walk(detection)
        if isinstance(node, ast.For)
        and isinstance(node.iter, ast.Name)
        and node.iter.id == "actions"
    )
    publish_lines = [
        call.lineno
        for call in ast.walk(action_loop)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "publish_control_outputs"
    ]
    kpu_lines = [
        call.lineno
        for call in ast.walk(action_loop)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "kpu"
        and call.func.attr == "run"
    ]
    assert publish_lines and kpu_lines
    assert min(publish_lines) < min(kpu_lines)


def test_blob_detector_exception_releases_frame_and_falls_back_to_kpu():
    events = []

    class Frame:
        def __del__(self):
            events.append("release")

    class Sensor:
        def snapshot(self, **kwargs):
            events.append(("snapshot", kwargs))
            return Frame()

    def failing_detector(*args):
        events.append("detect")
        raise RuntimeError("find_blobs failed")

    snapshot_blob_channel = load_pure_function(
        "snapshot_blob_channel",
        {
            "CAM_CHN_ID_1": 1,
            "detect_blob_measurement": failing_detector,
            "print": lambda *args: events.append("fallback"),
        },
    )
    schedule = load_pure_function("hybrid_frame_actions")

    capture, available = snapshot_blob_channel(
        Sensor(), True, (4, 110, 192, 140), 100, 180)

    assert capture is None
    assert available is False
    assert events == [
        ("snapshot", {"chn": 1, "timeout": 2000}),
        "detect",
        "fallback",
        "release",
    ]
    assert schedule("TRACK", 7, available) == ("kpu",)


def test_blob_channel_does_not_swallow_keyboard_interrupt():
    class Sensor:
        def snapshot(self, **kwargs):
            return object()

    def interrupt_detector(*args):
        raise KeyboardInterrupt()

    snapshot_blob_channel = load_pure_function(
        "snapshot_blob_channel",
        {
            "CAM_CHN_ID_1": 1,
            "detect_blob_measurement": interrupt_detector,
            "print": lambda *args: None,
        },
    )

    try:
        snapshot_blob_channel(
            Sensor(), True, (4, 110, 192, 140), 100, 180)
    except KeyboardInterrupt:
        return
    assert False, "KeyboardInterrupt was swallowed by Blob fallback"


def test_blob_channel_is_best_effort_and_configured_before_media_init():
    events = []
    sensors = []

    class FakeSensorType:
        RGB565 = "rgb565"

    class FakeSensor:
        def __init__(self, enable_blob_channel):
            self.enable_blob_channel = enable_blob_channel
            self.channels = set()
            sensors.append(self)

        def reset(self):
            events.append(("reset", self.enable_blob_channel))

        def set_hmirror(self, value):
            pass

        def set_vflip(self, value):
            pass

        def set_framesize(self, **kwargs):
            self.channels.add(kwargs.get("chn", 0))

        def set_pixformat(self, pixel_format, chn=0):
            self.channels.add(chn)

        def bind_info(self, **kwargs):
            events.append(("bind_info", self.enable_blob_channel))
            return {"src": self.enable_blob_channel}

        def run(self):
            events.append(("sensor_run", self.enable_blob_channel))
            if self.enable_blob_channel:
                raise RuntimeError("deferred channel 1 failure")

        def stop(self, is_del=False):
            events.append(("sensor_stop", self.enable_blob_channel, is_del))

    class FakeDisplay:
        ST7701 = "lcd"
        LT9611 = "hdmi"
        LAYER_VIDEO1 = 1

        @staticmethod
        def bind_layer(**kwargs):
            events.append(("display_bind", kwargs["src"]))

        @staticmethod
        def init(display_type, to_ide=False):
            events.append(("display_init", display_type))

        @staticmethod
        def width():
            return 800

        @staticmethod
        def height():
            return 480

        @staticmethod
        def deinit():
            events.append(("display_deinit",))

    class FakeMediaManager:
        @staticmethod
        def init():
            events.append(("media_init", sensors[-1].enable_blob_channel))

        @staticmethod
        def deinit():
            events.append(("media_deinit",))

    class FakeImage:
        ARGB8888 = "argb"

        @staticmethod
        def Image(width, height, pixel_format):
            events.append(("osd", width, height, pixel_format))
            return "osd"

    class FakeRtspServer:
        def __init__(self, width, height, port, session):
            events.append(("rtsp", width, height, port, session))

    configure_namespace = {
        "DISPLAY_WIDTH": 800,
        "DISPLAY_HEIGHT": 480,
        "OUT_RGB888P_WIDTH": 640,
        "OUT_RGB888P_HEIGH": 360,
        "PIXEL_FORMAT_YUV_SEMIPLANAR_420": "yuv420",
        "PIXEL_FORMAT_RGB_888_PLANAR": "rgbp888",
        "CAM_CHN_ID_1": 1,
        "CAM_CHN_ID_2": 2,
        "Sensor": FakeSensorType,
    }
    configure_camera_sensor = load_pure_function(
        "configure_camera_sensor", configure_namespace)

    def create_camera_sensor(enable_blob_channel):
        sensor = FakeSensor(enable_blob_channel)
        return configure_camera_sensor(sensor, enable_blob_channel)

    cleanup_camera_start = load_pure_function(
        "cleanup_camera_start",
        {"Display": FakeDisplay, "MediaManager": FakeMediaManager},
    )
    start_camera_pipeline = load_pure_function(
        "start_camera_pipeline",
        {
            "create_camera_sensor": create_camera_sensor,
            "cleanup_camera_start": cleanup_camera_start,
            "Display": FakeDisplay,
            "MediaManager": FakeMediaManager,
            "image": FakeImage,
            "LowLatencyRtspH264Server": FakeRtspServer,
            "display_mode": "lcd",
            "DISPLAY_WIDTH": 800,
            "DISPLAY_HEIGHT": 480,
            "CAM_CHN_ID_0": 0,
            "RTSP_PORT": 8554,
            "RTSP_SESSION": "ball",
        },
    )
    start_camera_with_blob_fallback = load_pure_function(
        "start_camera_with_blob_fallback", {"print": lambda *args: None})

    sensor, osd_img, rtsp_server, blob_available = (
        start_camera_with_blob_fallback(start_camera_pipeline))

    assert sensor is sensors[1]
    assert sensors[0] is not sensors[1]
    assert sensors[0].channels == {0, 1, 2}
    assert sensors[1].channels == {0, 2}
    assert osd_img == "osd"
    assert isinstance(rtsp_server, FakeRtspServer)
    assert blob_available is False
    assert events.index(("media_init", True)) < events.index(("sensor_run", True))
    assert events.index(("sensor_stop", True, True)) < events.index(("reset", False))
    assert events.index(("media_init", False)) < events.index(("sensor_run", False))


def test_blob_setter_failure_releases_partial_sensor_before_fallback():
    class FakeGc:
        @staticmethod
        def collect():
            pass

    class FakeTime:
        @staticmethod
        def sleep_ms(duration):
            pass

    class LegacySensor:
        RGB565 = "rgb565"
        active = False

        def __init__(self, id, fps):
            if LegacySensor.active:
                raise RuntimeError("sensor already initialized")
            LegacySensor.active = True

        def reset(self):
            pass

        def set_hmirror(self, value):
            pass

        def set_vflip(self, value):
            pass

        def set_framesize(self, **kwargs):
            pass

        def set_pixformat(self, pixel_format, chn=0):
            if chn == 1:
                raise RuntimeError("channel 1 unsupported")

        def stop(self, *args, **kwargs):
            if kwargs:
                raise TypeError("legacy stop has no is_del")
            LegacySensor.active = False

    constants = {
        "DISPLAY_WIDTH": 800,
        "DISPLAY_HEIGHT": 480,
        "OUT_RGB888P_WIDTH": 640,
        "OUT_RGB888P_HEIGH": 360,
        "PIXEL_FORMAT_YUV_SEMIPLANAR_420": "yuv420",
        "PIXEL_FORMAT_RGB_888_PLANAR": "rgbp888",
        "CAM_CHN_ID_1": 1,
        "CAM_CHN_ID_2": 2,
        "CAMERA_PROBE_RETRIES": 1,
        "CAMERA_CSI_ID": 2,
        "Sensor": LegacySensor,
        "gc": FakeGc,
        "time": FakeTime,
    }
    configure_camera_sensor = load_pure_function(
        "configure_camera_sensor", constants)
    cleanup_camera_start = load_pure_function(
        "cleanup_camera_start",
        {"Display": None, "MediaManager": None},
    )
    create_camera_sensor = load_pure_function(
        "create_camera_sensor",
        dict(
            constants,
            configure_camera_sensor=configure_camera_sensor,
            cleanup_camera_start=cleanup_camera_start,
        ),
    )
    start_camera_with_blob_fallback = load_pure_function(
        "start_camera_with_blob_fallback", {"print": lambda *args: None})

    def start_attempt(enable_blob_channel):
        return create_camera_sensor(enable_blob_channel), "osd", "rtsp"

    sensor, _, _, blob_available = start_camera_with_blob_fallback(start_attempt)
    assert isinstance(sensor, LegacySensor)
    assert blob_available is False


def test_three_sample_velocity_and_bounded_prediction():
    estimate_velocity = load_pure_function("estimate_velocity")
    predict_position = load_pure_function("predict_position")
    samples = [(100, 50, 0), (104, 50, 20), (110, 52, 40)]
    vx, vy = estimate_velocity(samples)
    assert round(vx, 3) == 0.25
    assert round(vy, 3) == 0.05
    assert predict_position(110, 52, vx, vy, 40, 16) == (120, 54, False)
    assert predict_position(110, 52, 1.0, 0.0, 40, 16) == (126, 52, True)


def test_osd_updates_at_the_requested_cadence():
    should_render_osd = load_pure_function("should_render_osd")
    assert should_render_osd(1, 1) is True
    assert should_render_osd(2, 1) is True
    assert should_render_osd(3, 1) is True
    assert should_render_osd(4, 1) is True


def test_control_ui_is_full_rate_and_rtc_is_removed():
    function_names = {
        item.name for item in TREE.body if isinstance(item, ast.FunctionDef)
    }
    assert "format_iso_time" not in function_names
    text = SOURCE.read_text(encoding="utf-8")
    assert "System time:" not in text
    assert "RTC time is not calibrated" not in text
    assert "OSD_EVERY_N_FRAMES       = 1" in text


def test_uart_updates_are_not_gated_by_osd_rendering():
    draw_osd = next(
        item
        for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == "draw_osd"
    )
    for branch in (node for node in ast.walk(draw_osd) if isinstance(node, ast.If)):
        if not any(
            isinstance(name, ast.Name) and name.id == "render_osd"
            for name in ast.walk(branch.test)
        ):
            continue
        assert not any(
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "write"
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == "uart_obj"
            for stmt in branch.body
            for call in ast.walk(stmt)
        )


def test_h264_rtsp_replaces_mjpeg_transport():
    class_names = {
        item.name for item in TREE.body if isinstance(item, ast.ClassDef)
    }
    assert "LowLatencyRtspH264Server" in class_names
    assert "LowLatencyMjpegServer" not in class_names

    assignments = {
        target.id: ast.literal_eval(item.value)
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name)
        and target.id in {"RTSP_PORT", "RTSP_SESSION", "H264_FPS"}
    }
    assert assignments == {
        "RTSP_PORT": 8554,
        "RTSP_SESSION": "ball",
        "H264_FPS": 30,
    }


def test_control_interface_and_model_paths_are_unchanged():
    format_deviation_msg = load_pure_function(
        "format_deviation_msg", {"DEVIATION_DEADZONE": 3})
    assert format_deviation_msg(12, -7, True) == b"X:+012,Y:-007\n"
    assert format_deviation_msg(2, -3, True) == b"X:+000,Y:+000\n"
    assert format_deviation_msg(0, 0, False) == b"X:----,Y:----\n"

    assignments = {
        target.id: ast.literal_eval(item.value)
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name)
        and target.id in {
            "root_path", "config_path", "UART_BAUDRATE",
            "SEND_EVERY_N_FRAMES",
        }
    }
    assert assignments == {
        "root_path": "/sdcard/mp_deployment_source/",
        "config_path": "/sdcard/mp_deployment_source/deploy_config.json",
        "UART_BAUDRATE": 115200,
        "SEND_EVERY_N_FRAMES": 1,
    }


def test_rtsp_void_and_zero_returns_are_successful():
    rtsp_call_succeeded = load_pure_function("rtsp_call_succeeded")
    assert rtsp_call_succeeded(None) is True
    assert rtsp_call_succeeded(0) is True
    assert rtsp_call_succeeded(-1) is False


def test_rtsp_worker_isolated_from_control_outputs_and_startup_is_guarded():
    rtsp_server = next(
        item
        for item in TREE.body
        if isinstance(item, ast.ClassDef)
        and item.name == "LowLatencyRtspH264Server"
    )
    methods = {
        item.name: item
        for item in rtsp_server.body
        if isinstance(item, ast.FunctionDef)
    }
    start = methods["start"]

    thread_calls = [
        call
        for call in ast.walk(start)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "_thread"
        and call.func.attr == "start_new_thread"
    ]
    assert len(thread_calls) == 1
    assert isinstance(thread_calls[0].args[0], ast.Attribute)
    assert isinstance(thread_calls[0].args[0].value, ast.Name)
    assert thread_calls[0].args[0].value.id == "self"
    assert thread_calls[0].args[0].attr == "_stream_loop"

    guarded_thread_start = any(
        thread_calls[0] in ast.walk(statement)
        for branch in ast.walk(start)
        if isinstance(branch, ast.Try) and branch.handlers
        for statement in branch.body
    )
    assert guarded_thread_start

    assert rtsp_worker_control_calls(
        TREE, "LowLatencyRtspH264Server", "_stream_loop") == set()


def test_rtsp_call_graph_detects_module_alias_and_uart_wrappers():
    tree = ast.parse(
        """
def publish_wrapper():
    publish_measurement(1, 2, "blob", 3)

def uart_wrapper(uart):
    uart.write(b"X:+001,Y:+002\\n")

callback = publish_wrapper

class Worker:
    def _measurement_helper(self):
        callback()

    def _uart_helper(self, uart):
        uart_wrapper(uart)

    def _stream_loop(self, uart):
        self._measurement_helper()
        Worker._uart_helper(self, uart)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {
            "publish_measurement", "uart.write",
        }


def test_rtsp_call_graph_allows_network_writer():
    tree = ast.parse(
        """
def send_packet(client):
    client.write(b"video")

class Worker:
    def _stream_loop(self, client):
        send_packet(client)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == set()


def test_rtsp_call_graph_detects_direct_boundary_alias():
    tree = ast.parse(
        """
callback = publish_measurement

class Worker:
    def _stream_loop(self):
        callback(1, 2, "blob", 3)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"publish_measurement"}


def test_rtsp_call_graph_tracks_uart_argument_provenance():
    tree = ast.parse(
        """
def write_port(port):
    port.write(b"X:+001,Y:+002\\n")

class Worker:
    def _stream_loop(self, uart):
        write_port(port=uart)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"uart.write"}


def test_rtsp_call_graph_tracks_positional_uart_provenance():
    tree = ast.parse(
        """
def write_port(port):
    port.write(b"X:+001,Y:+002\\n")

class Worker:
    def _stream_loop(self, uart):
        write_port(uart)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"uart.write"}


def test_rtsp_call_graph_keeps_network_argument_provenance_allowed():
    tree = ast.parse(
        """
def write_port(port):
    port.write(b"video")

class Worker:
    def _stream_loop(self, client):
        write_port(client)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == set()


def test_rtsp_call_graph_respects_nested_function_scope_and_invocation():
    not_called_tree = ast.parse(
        """
class Worker:
    def _stream_loop(self):
        def hidden_control():
            publish_measurement(1, 2, "blob", 3)
        return None
""")
    assert rtsp_worker_control_calls(
        not_called_tree, "Worker", "_stream_loop") == set()

    called_tree = ast.parse(
        """
class Worker:
    def _stream_loop(self):
        def hidden_control():
            publish_measurement(1, 2, "blob", 3)
        hidden_control()
""")
    assert rtsp_worker_control_calls(
        called_tree, "Worker", "_stream_loop") == {"publish_measurement"}


def test_rtsp_call_graph_resolves_aliases_at_each_call_site():
    tree = ast.parse(
        """
def send_packet():
    return None

class Worker:
    def _stream_loop(self):
        callback = publish_measurement
        callback()
        callback = send_packet
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"publish_measurement"}


def test_rtsp_call_graph_maps_bound_method_arguments_after_self():
    tree = ast.parse(
        """
class Worker:
    def helper(self, port):
        port.write(b"X:+001,Y:+002\\n")

    def _stream_loop(self, uart):
        self.helper(uart)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"uart.write"}


def test_rtsp_call_graph_ignores_self_aliases_from_uncalled_methods():
    tree = ast.parse(
        """
class Worker:
    def unused(self):
        self.callback = publish_measurement

    def _stream_loop(self):
        self.callback()
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == set()


def test_rtsp_call_graph_carries_reachable_self_alias_state_in_order():
    tree = ast.parse(
        """
def send_packet():
    return None

class Worker:
    def configure(self):
        self.callback = publish_measurement

    def _stream_loop(self):
        self.configure()
        self.callback()

    def unrelated(self):
        self.callback = send_packet
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"publish_measurement"}


def test_rtsp_call_graph_indexes_invoked_nested_defs_in_compound_blocks():
    tree = ast.parse(
        """
class Worker:
    def _stream_loop(self):
        if enabled:
            def from_if():
                publish_measurement()

            def not_called():
                draw_osd()

        try:
            def from_try():
                update_servo_control()
        except BaseException:
            pass

        for item in items:
            def from_loop():
                invalidate_control_state()

        from_if()
        from_try()
        from_loop()
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {
            "publish_measurement", "update_servo_control",
            "invalidate_control_state",
        }


def test_rtsp_call_graph_does_not_taint_callee_from_uart_parameter_name():
    tree = ast.parse(
        """
def send(uart):
    uart.write(b"video")

class Worker:
    def _stream_loop(self, client):
        send(client)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == set()


def test_rtsp_call_graph_uses_late_bound_nested_function_aliases():
    boundary_tree = ast.parse(
        """
def send_packet():
    return None

class Worker:
    def _stream_loop(self):
        callback = send_packet

        def invoke():
            callback()

        callback = publish_measurement
        invoke()
""")
    assert rtsp_worker_control_calls(
        boundary_tree, "Worker", "_stream_loop") == {
            "publish_measurement",
        }

    safe_tree = ast.parse(
        """
def send_packet():
    return None

class Worker:
    def _stream_loop(self):
        callback = publish_measurement

        def invoke():
            callback()

        callback = send_packet
        invoke()
""")
    assert rtsp_worker_control_calls(
        safe_tree, "Worker", "_stream_loop") == set()


def test_rtsp_call_graph_keeps_foreign_instance_state_separate():
    tree = ast.parse(
        """
class Other:
    def configure(self):
        self.callback = publish_measurement

class Worker:
    def _stream_loop(self, client):
        Other.configure(client)
        self.callback()
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == set()


def test_rtsp_call_graph_seeds_known_module_uart_roots():
    tree = ast.parse(
        """
uart = UART(1)

def send_control():
    uart.write(b"X:+001,Y:+002\\n")

class Worker:
    def _stream_loop(self):
        send_control()
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"uart.write"}


def test_rtsp_call_graph_keeps_lexical_closure_through_helper_invocation():
    boundary_tree = ast.parse(
        """
def invoke_helper(callback):
    callback()

def send_packet():
    return None

class Worker:
    def _stream_loop(self):
        callback = send_packet

        def invoke():
            callback()

        callback = publish_measurement
        invoke_helper(invoke)
""")
    assert rtsp_worker_control_calls(
        boundary_tree, "Worker", "_stream_loop") == {
            "publish_measurement",
        }

    safe_tree = ast.parse(
        """
def invoke_helper(callback):
    callback()

def send_packet():
    return None

class Worker:
    def _stream_loop(self):
        callback = publish_measurement

        def invoke():
            callback()

        callback = send_packet
        invoke_helper(invoke)
""")
    assert rtsp_worker_control_calls(
        safe_tree, "Worker", "_stream_loop") == set()


def test_rtsp_call_graph_preserves_foreign_state_within_nested_calls():
    tree = ast.parse(
        """
class Other:
    def configure(self):
        self.callback = publish_measurement

    def run(self):
        self.configure()
        self.callback()

class Worker:
    def _stream_loop(self, client):
        Other.run(client)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"publish_measurement"}


def test_rtsp_call_graph_keeps_captured_self_state_through_helper():
    tree = ast.parse(
        """
def invoke_helper(callback):
    callback()

class Worker:
    def _stream_loop(self):
        self.callback = publish_measurement

        def invoke():
            self.callback()

        invoke_helper(invoke)
""")
    assert rtsp_worker_control_calls(
        tree, "Worker", "_stream_loop") == {"publish_measurement"}


def test_low_rate_metrics_use_scalar_counters_and_interval_only_formatting():
    assignments = {
        target.id: ast.literal_eval(item.value)
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name)
        and target.id in {
            "METRICS_EVERY_N_CONTROL_FRAMES",
            "blob_frame_count", "blob_total_ms",
            "kpu_validation_count", "kpu_total_ms",
            "blob_loss_count", "kpu_reacquire_count",
            "prediction_clamp_count",
        }
    }
    assert assignments == {
        "METRICS_EVERY_N_CONTROL_FRAMES": 60,
        "blob_frame_count": 0,
        "blob_total_ms": 0,
        "kpu_validation_count": 0,
        "kpu_total_ms": 0,
        "blob_loss_count": 0,
        "kpu_reacquire_count": 0,
        "prediction_clamp_count": 0,
    }

    function_names = {
        item.name for item in TREE.body if isinstance(item, ast.FunctionDef)
    }
    assert "tracking_metrics_report" in function_names

    detection = next(
        item
        for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == "detection"
    )
    report_calls = [
        call
        for call in ast.walk(detection)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "tracking_metrics_report"
    ]
    assert len(report_calls) == 1

    metric_guards = [
        branch
        for branch in ast.walk(detection)
        if isinstance(branch, ast.If)
        and any(
            isinstance(name, ast.Name)
            and name.id == "METRICS_EVERY_N_CONTROL_FRAMES"
            for name in ast.walk(branch.test)
        )
    ]
    assert len(metric_guards) == 1
    assert report_calls[0] in ast.walk(metric_guards[0])


def test_tracking_metrics_report_uses_exact_cadence_and_resets_window():
    class FakeTime:
        @staticmethod
        def ticks_diff(now_ms, start_ms):
            return now_ms - start_ms

    report = load_pure_function(
        "tracking_metrics_report", {"time": FakeTime})
    assert report(
        59, "TRACK", 160, 100,
        3, 30, 2, 20, 4, 5, 6) is None
    assert report(
        61, "TRACK", 160, 100,
        3, 30, 2, 20, 4, 5, 6) is None
    result = report(
        60, "TRACK", 160, 100,
        3, 30, 2, 20, 4, 5, 6)
    assert result == (
        "TRACK:TRACK CTRL:1000.0 Blob:10.0 KPU:10.0 "
        "Lost:4 Reacq:5 Clamp:6",
        160, 0, 0, 0, 0, 0, 0, 0,
    )


def test_tracking_metrics_report_guards_empty_averages_and_uses_ticks_diff():
    class FakeTime:
        calls = []

        @staticmethod
        def ticks_diff(now_ms, start_ms):
            FakeTime.calls.append((now_ms, start_ms))
            return 25

    report = load_pure_function(
        "tracking_metrics_report", {"time": FakeTime})
    result = report(
        120, "RECOVER", 3, 0xfffffff0,
        0, 0, 0, 0, 0, 0, 0)
    assert result[0] == (
        "TRACK:RECOVER CTRL:2400.0 Blob:0.0 KPU:0.0 "
        "Lost:0 Reacq:0 Clamp:0")
    assert FakeTime.calls == [(3, 0xfffffff0)]


def test_kpu_latency_metrics_only_measure_track_validation_processing():
    detection = next(
        item
        for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == "detection"
    )
    validation_guards = [
        branch
        for branch in ast.walk(detection)
        if isinstance(branch, ast.If)
        and any(
            isinstance(name, ast.Name) and name.id == "is_kpu_validation"
            for name in ast.walk(branch.test)
        )
    ]
    assert validation_guards
    validation_updates = {
        target.id
        for branch in validation_guards
        for node in ast.walk(branch)
        if isinstance(node, ast.AugAssign)
        and isinstance(node.target, ast.Name)
        and node.target.id in {"kpu_validation_count", "kpu_total_ms"}
        for target in (node.target,)
    }
    assert validation_updates == {"kpu_validation_count", "kpu_total_ms"}

    first_ai_startup = next(
        branch
        for branch in ast.walk(detection)
        if isinstance(branch, ast.If)
        and isinstance(branch.test, ast.Name)
        and branch.test.id == "first_ai_frame"
    )
    kpu_timing_starts = [
        node
        for node in ast.walk(detection)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "kpu_start_ms"
            for target in node.targets
        )
    ]
    assert len(kpu_timing_starts) == 1
    assert kpu_timing_starts[0].lineno > first_ai_startup.end_lineno


def test_wifi_scan_supports_firmware_info_objects():
    wifi_scan_channel_rssi = load_pure_function("wifi_scan_channel_rssi")

    class ScanInfo:
        channel = 6
        rssi = -42

    assert wifi_scan_channel_rssi(ScanInfo()) == (6, -42)
    assert wifi_scan_channel_rssi({"channel": 11, "rssi": -70}) == (11, -70)
    assert wifi_scan_channel_rssi(("ssid", b"mac", 1, -55, 0, 0)) == (1, -55)


if __name__ == "__main__":
    test_detection_circle_geometry()
    test_single_ai_capture_selects_highest_valid_confidence()
    test_single_ai_capture_rejects_invalid_configured_boxes()
    test_blob_candidate_prefers_nearest_valid_ball()
    test_blob_detection_returns_plain_candidate_from_requested_roi()
    test_blob_detection_uses_predicted_center_when_roi_is_edge_clamped()
    test_blob_tracking_roi_stays_inside_the_global_rod_region()
    test_hybrid_tracking_transitions()
    test_kpu_reacquisition_clears_blob_misses_before_returning_to_track()
    test_kpu_capture_within_blob_identity_gate_is_valid()
    test_distant_kpu_capture_fails_without_overwriting_blob_control()
    test_hybrid_schedule_is_control_first_with_six_frame_validation()
    test_one_missed_blob_frame_is_predicted_then_requests_invalidation()
    test_tracking_marker_uses_state_color_and_skips_invalid_control()
    test_control_outputs_call_servo_hook_before_uart_osd_display()
    test_kpu_frame_resources_are_released_before_blob_only_frames()
    test_validation_loop_commits_control_before_kpu_inference()
    test_blob_detector_exception_releases_frame_and_falls_back_to_kpu()
    test_blob_channel_does_not_swallow_keyboard_interrupt()
    test_blob_channel_is_best_effort_and_configured_before_media_init()
    test_blob_setter_failure_releases_partial_sensor_before_fallback()
    test_three_sample_velocity_and_bounded_prediction()
    test_osd_updates_at_the_requested_cadence()
    test_control_ui_is_full_rate_and_rtc_is_removed()
    test_uart_updates_are_not_gated_by_osd_rendering()
    test_h264_rtsp_replaces_mjpeg_transport()
    test_control_interface_and_model_paths_are_unchanged()
    test_rtsp_void_and_zero_returns_are_successful()
    test_rtsp_worker_isolated_from_control_outputs_and_startup_is_guarded()
    test_rtsp_call_graph_detects_module_alias_and_uart_wrappers()
    test_rtsp_call_graph_allows_network_writer()
    test_rtsp_call_graph_detects_direct_boundary_alias()
    test_rtsp_call_graph_tracks_uart_argument_provenance()
    test_rtsp_call_graph_tracks_positional_uart_provenance()
    test_rtsp_call_graph_keeps_network_argument_provenance_allowed()
    test_rtsp_call_graph_respects_nested_function_scope_and_invocation()
    test_rtsp_call_graph_resolves_aliases_at_each_call_site()
    test_rtsp_call_graph_maps_bound_method_arguments_after_self()
    test_rtsp_call_graph_ignores_self_aliases_from_uncalled_methods()
    test_rtsp_call_graph_carries_reachable_self_alias_state_in_order()
    test_rtsp_call_graph_indexes_invoked_nested_defs_in_compound_blocks()
    test_rtsp_call_graph_does_not_taint_callee_from_uart_parameter_name()
    test_rtsp_call_graph_uses_late_bound_nested_function_aliases()
    test_rtsp_call_graph_keeps_foreign_instance_state_separate()
    test_rtsp_call_graph_seeds_known_module_uart_roots()
    test_rtsp_call_graph_keeps_lexical_closure_through_helper_invocation()
    test_rtsp_call_graph_preserves_foreign_state_within_nested_calls()
    test_rtsp_call_graph_keeps_captured_self_state_through_helper()
    test_low_rate_metrics_use_scalar_counters_and_interval_only_formatting()
    test_tracking_metrics_report_uses_exact_cadence_and_resets_window()
    test_tracking_metrics_report_guards_empty_averages_and_uses_ticks_diff()
    test_kpu_latency_metrics_only_measure_track_validation_processing()
    test_wifi_scan_supports_firmware_info_objects()
    print("tests: OK")
