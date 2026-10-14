import json
import os
import shlex
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path
from venv import EnvBuilder

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/start_mcp.sh"


@pytest.fixture
def startup_environment(tmp_path):
    ros = tmp_path / "ROS installation"
    workspace = tmp_path / "robot workspace"
    venv = tmp_path / "Python environment"
    modules = tmp_path / "modules"
    ros.mkdir()
    (workspace / "install").mkdir(parents=True)
    (venv / "bin").mkdir(parents=True)
    (venv / "bin/python").symlink_to(sys.executable)
    (venv / "bin/activate").write_text(f"export VIRTUAL_ENV={shlex.quote(str(venv))}\n")
    for shell in ("bash", "zsh"):
        (ros / f"setup.{shell}").write_text(
            f"export EE_MCP_ROS_SETUP={shell}\nexport ROS_DISTRO=humble\n"
            f"export PYTHONPATH={shlex.quote(str(modules))}\n"
        )
        (workspace / f"install/setup.{shell}").write_text(
            f"export EE_MCP_WORKSPACE_SETUP={shell}\n"
            "export ROS_DOMAIN_ID=99\nexport ROS_LOCALHOST_ONLY=1\n"
        )
    # Replace Python imports and the final server only, so no DDS participant
    # or physical control publisher can be created while testing the launcher.
    for package in ("rclpy", "sensor_msgs", "arx5_arm_msg", "ee_control_mcp"):
        (modules / package).mkdir(parents=True)
        (modules / package / "__init__.py").touch()
    # Use the real config parser, while keeping ROS and the final server stubbed.
    (modules / "ee_control_mcp/__init__.py").write_text(
        f"__path__.append({str(ROOT / 'src/ee_control_mcp')!r})\n"
    )
    for package, content in (
        ("sensor_msgs", "class CompressedImage: pass\n"),
        ("arx5_arm_msg", "class RobotCmd: pass\nclass RobotStatus: pass\n"),
    ):
        (modules / package / "msg").mkdir()
        (modules / package / "msg/__init__.py").write_text(content)
    (modules / "ee_control_mcp/server.py").touch()
    (modules / "ee_control_mcp/__main__.py").write_text(
        "import json, os, sys\n"
        "print('START_RECORD=' + json.dumps({'argv':sys.argv[1:],'cwd':os.getcwd(),"
        "'env':{name:os.environ.get(name) for name in ['EE_MCP_ROS_SETUP',"
        "'EE_MCP_WORKSPACE_SETUP','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','VIRTUAL_ENV']},"
        "'python':sys.executable,'conda_prefix':os.environ.get('CONDA_PREFIX'),"
        "'pythonpath':os.environ.get('PYTHONPATH')}))\n"
        "sys.exit(int(os.environ.get('EE_MCP_TEST_EXIT_CODE','0')))\n"
    )
    env = {**os.environ, "PYTHONPATH": str(modules)}
    env.pop("PYTHONHOME", None)
    return ros, workspace, venv, modules, env


def run_launcher(
    startup_environment, shell="bash", extra=(), *, direct=False, environment_options=None
):
    ros, workspace, venv, _, env = startup_environment
    if shutil.which(shell) is None:
        pytest.skip(f"{shell} unavailable")
    args = [
        str(SCRIPT),
        "--ros-dir",
        str(ros),
        "--workspace",
        str(workspace),
        *(environment_options if environment_options is not None else ["--venv", str(venv)]),
        *extra,
    ]
    env = {**env, "SHELL": "/bin/zsh" if shell == "bash" else "/bin/bash"}
    if direct:
        # Keep the real caller shell alive so its identity is visible to PPID.
        command = shlex.join(args) + "\nmcp_exit=$?\nexit $mcp_exit"
        argv = [shell, "-c", command]
    else:
        argv = [shell, *args]
    return subprocess.run(argv, env=env, cwd=workspace, text=True, capture_output=True, timeout=10)


def record(result):
    assert result.returncode == 0, result.stdout + result.stderr
    return json.loads(
        next(
            line.removeprefix("START_RECORD=")
            for line in result.stdout.splitlines()
            if line.startswith("START_RECORD=")
        )
    )


@pytest.mark.parametrize("shell", ["bash", "zsh"])
@pytest.mark.parametrize(
    "mode,readonly", [((), True), (("--readonly",), True), (("--control",), False)]
)
def test_launcher_loads_matching_environments_and_enforces_mode(
    startup_environment, shell, mode, readonly
):
    data = record(run_launcher(startup_environment, shell, (*mode, "--port", "3012")))
    assert data["env"] == {
        "EE_MCP_ROS_SETUP": shell,
        "EE_MCP_WORKSPACE_SETUP": shell,
        "ROS_DOMAIN_ID": "60",
        "ROS_LOCALHOST_ONLY": "0",
        "VIRTUAL_ENV": str(startup_environment[2]),
    }
    assert data["cwd"] == str(ROOT)
    assert ("--no-actuation" in data["argv"]) is readonly
    assert data["argv"][data["argv"].index("--backend") + 1] == "ros"
    assert data["argv"][data["argv"].index("--port") + 1] == "3012"


@pytest.mark.parametrize("shell", ["bash", "zsh"])
def test_executable_launcher_detects_parent_shell_instead_of_login_default(
    startup_environment, shell
):
    data = record(run_launcher(startup_environment, shell, direct=True))
    assert data["env"]["EE_MCP_ROS_SETUP"] == shell
    assert data["env"]["EE_MCP_WORKSPACE_SETUP"] == shell


def test_missing_workspace_install_does_not_start_server(startup_environment):
    (startup_environment[1] / "install/setup.bash").unlink()
    result = run_launcher(startup_environment)
    assert result.returncode != 0 and "--build-messages" in result.stderr
    assert "START_RECORD=" not in result.stdout


@pytest.mark.parametrize(
    "args",
    [
        ("--control", "--readonly"),
        ("--readonly", "--control"),
        ("--workspace",),
        ("--unknown",),
    ],
)
def test_invalid_launch_options_are_rejected_before_startup(startup_environment, args):
    result = run_launcher(startup_environment, extra=args)
    assert result.returncode == 2
    assert "START_RECORD=" not in result.stdout
    if "--control" in args and "--readonly" in args:
        assert "不能同时使用" in result.stderr


def test_missing_virtualenv_is_reported_before_startup(startup_environment):
    (startup_environment[2] / "bin/python").unlink()
    result = run_launcher(startup_environment)
    assert result.returncode != 0 and "虚拟环境" in result.stderr
    assert "START_RECORD=" not in result.stdout


@pytest.mark.parametrize("mode", ["--control", "--readonly"])
def test_robotcmd_dependency_is_required_only_for_control(startup_environment, mode):
    (startup_environment[3] / "arx5_arm_msg/msg/__init__.py").write_text(
        "class RobotStatus: pass\n"
    )
    result = run_launcher(startup_environment, extra=(mode,))
    if mode == "--control":
        assert result.returncode != 0 and "依赖" in result.stderr
        assert "START_RECORD=" not in result.stdout
    else:
        assert "--no-actuation" in record(result)["argv"]


@pytest.mark.parametrize("shell", ["bash", "zsh"])
def test_sourcing_launcher_preserves_original_shell(startup_environment, shell):
    if shutil.which(shell) is None:
        pytest.skip(f"{shell} unavailable")
    result = subprocess.run(
        [
            shell,
            "-c",
            f"source {shlex.quote(str(SCRIPT))}\n"
            "mcp_result=$?\nprintf 'SOURCE_STATUS=%s\\n' \"$mcp_result\"",
        ],
        env=startup_environment[4],
        text=True,
        capture_output=True,
        timeout=10,
    )
    assert result.returncode == 0 and "SOURCE_STATUS=2" in result.stdout
    assert "不要 source" in result.stderr
    assert "START_RECORD=" not in result.stdout


def test_launcher_preserves_server_exit_code(startup_environment):
    startup_environment[4]["EE_MCP_TEST_EXIT_CODE"] = "7"
    result = run_launcher(startup_environment)
    assert result.returncode == 7 and "START_RECORD=" in result.stdout


@pytest.fixture
def conda_environment(startup_environment, tmp_path):
    base = tmp_path / "Conda installation"
    prefix = tmp_path / "MCP Conda environment"
    (base / "bin").mkdir(parents=True)
    (base / "etc/profile.d").mkdir(parents=True)
    # Give the selected interpreter its own package search path, just like the
    # deployed environment, so it can load config before the ROS setup runs.
    EnvBuilder(with_pip=False, system_site_packages=True).create(prefix)
    site_packages = (
        prefix / f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    )
    (site_packages / "mcp-test-dependencies.pth").write_text(
        str(ROOT / "src") + "\n" + sysconfig.get_path("purelib") + "\n"
    )
    conda = base / "bin/conda"
    conda.write_text(
        "#!/bin/sh\n"
        '[ "$1" = info ] && [ "$2" = --base ] || exit 3\n'
        f"printf '%s\\n' {shlex.quote(str(base))}\n"
    )
    conda.chmod(0o755)
    # Only replace Conda's shell activation boundary; execute the real launcher
    # and Python with stubbed ROS modules to avoid any DDS/robot connection.
    (base / "etc/profile.d/conda.sh").write_text(
        "conda() {\n"
        '    [[ "$1" == activate ]] || return 3\n'
        '    case "$2" in\n'
        f"        {shlex.quote(str(prefix))}|ee-control-mcp) "
        f"export CONDA_PREFIX={shlex.quote(str(prefix))} ;;\n"
        f"        base|{shlex.quote(str(base))}) "
        f"export CONDA_PREFIX={shlex.quote(str(base))} ;;\n"
        "        *) return 3 ;;\n"
        "    esac\n"
        '    export PATH="$CONDA_PREFIX/bin:$PATH"\n'
        "    unset VIRTUAL_ENV\n"
        "}\n"
    )
    # Verify activation replaces a caller's pre-existing Python environment.
    startup_environment[4]["CONDA_PREFIX"] = str(base)
    startup_environment[4]["VIRTUAL_ENV"] = "/unrelated/venv"
    return base, prefix, conda


@pytest.mark.parametrize("shell", ["bash", "zsh"])
@pytest.mark.parametrize("named", [False, True])
def test_launcher_activates_selected_conda_environment(
    startup_environment, conda_environment, shell, named
):
    _, prefix, conda = conda_environment
    # There is no venv to fall back to: all Python invocations must use Conda.
    shutil.rmtree(startup_environment[2])
    options = ["--conda-env", "ee-control-mcp" if named else str(prefix), "--conda-exe", str(conda)]
    data = record(run_launcher(startup_environment, shell, environment_options=options))
    assert data["python"] == str(prefix / "bin/python")
    assert data["conda_prefix"] == str(prefix)
    assert data["env"]["VIRTUAL_ENV"] is None
    assert data["env"]["EE_MCP_ROS_SETUP"] == shell
    assert data["env"]["EE_MCP_WORKSPACE_SETUP"] == shell
    assert "--no-actuation" in data["argv"]


@pytest.mark.parametrize("use_path", [False, True])
def test_launcher_refuses_conda_base(startup_environment, conda_environment, use_path):
    base, _, conda = conda_environment
    result = run_launcher(
        startup_environment,
        environment_options=[
            "--conda-env",
            str(base) if use_path else "base",
            "--conda-exe",
            str(conda),
        ],
    )
    assert result.returncode != 0 and "base" in result.stderr
    assert "START_RECORD=" not in result.stdout


def test_failed_conda_activation_does_not_start_server(startup_environment, conda_environment):
    result = run_launcher(
        startup_environment,
        environment_options=["--conda-env", "missing", "--conda-exe", str(conda_environment[2])],
    )
    assert result.returncode != 0 and "激活 Conda" in result.stderr
    assert "START_RECORD=" not in result.stdout


def test_conda_and_venv_cannot_be_requested_together(startup_environment, conda_environment):
    result = run_launcher(
        startup_environment,
        extra=["--conda-env", str(conda_environment[1]), "--conda-exe", str(conda_environment[2])],
    )
    assert result.returncode == 2 and "不能同时" in result.stderr
    assert "START_RECORD=" not in result.stdout


@pytest.mark.parametrize("shell", ["bash", "zsh"])
@pytest.mark.parametrize("pollution", ["PYTHONPATH", "PYTHONHOME"])
def test_conda_launch_discards_caller_python_paths_and_reloads_ros(
    startup_environment, conda_environment, tmp_path, shell, pollution
):
    _, prefix, conda = conda_environment
    foreign = tmp_path / "foreign Python environment"
    (foreign / "pydantic_core").mkdir(parents=True)
    (foreign / "pydantic_core/__init__.py").write_text(
        "raise ImportError('foreign Python ABI loaded into Conda')\n"
    )
    (foreign / "ee_control_mcp").mkdir()
    (foreign / "ee_control_mcp/__init__.py").write_text(
        "raise ImportError('foreign environment loaded into MCP')\n"
    )
    startup_environment[4][pollution] = str(foreign)
    # Reproduce the real boundary: the Conda command runs its own Python, which
    # must not import an ABI-specific package from the caller's Python path.
    conda.write_text(
        "#!/bin/sh\n"
        '[ "$1" = info ] && [ "$2" = --base ] || exit 3\n'
        f"{shlex.quote(sys.executable)} -c 'import pydantic_core' || exit 3\n"
        f"printf '%s\\n' {shlex.quote(str(conda_environment[0]))}\n"
    )
    data = record(
        run_launcher(
            startup_environment,
            shell,
            environment_options=["--conda-env", str(prefix), "--conda-exe", str(conda)],
        )
    )
    assert data["python"] == str(prefix / "bin/python")
    assert data["pythonpath"] == str(startup_environment[3])
    assert data["env"]["EE_MCP_ROS_SETUP"] == shell
    assert data["env"]["EE_MCP_WORKSPACE_SETUP"] == shell


def test_optional_build_compiles_only_message_package_then_loads_install(startup_environment):
    _, workspace, venv, _, env = startup_environment
    shutil.rmtree(workspace / "install")
    tools = workspace / "tools"
    tools.mkdir()
    colcon = tools / "colcon"
    colcon.write_text(
        f"#!{sys.executable}\n"
        "import json, sys\nfrom pathlib import Path\n"
        "Path('build-record.json').write_text(json.dumps(sys.argv[1:]))\n"
        "install = Path(sys.argv[sys.argv.index('--install-base')+1]) "
        "if '--install-base' in sys.argv else Path('install')\n"
        "install.mkdir()\n"
        "for shell in ('bash','zsh'):\n"
        "    (install / f'setup.{shell}').write_text("
        "f'export EE_MCP_WORKSPACE_SETUP={shell}\\n')\n"
    )
    colcon.chmod(0o755)
    env["PATH"] = str(tools) + os.pathsep + env["PATH"]
    data = record(run_launcher(startup_environment, extra=("--build-messages",)))
    built = json.loads((workspace / "build-record.json").read_text())
    assert built == [
        "--log-base",
        "log-mcp-humble",
        "build",
        "--build-base",
        "build-mcp-humble",
        "--install-base",
        "install-mcp-humble",
        "--packages-select",
        "arx5_arm_msg",
        "--cmake-args",
        f"-DPython3_EXECUTABLE={venv / 'bin/python'}",
    ]
    assert data["env"]["EE_MCP_WORKSPACE_SETUP"] == "bash"
    assert not (workspace / "install").exists()


def test_failed_message_build_does_not_start_server(startup_environment):
    _, workspace, _, _, env = startup_environment
    tools = workspace / "tools"
    tools.mkdir()
    colcon = tools / "colcon"
    colcon.write_text("#!/bin/sh\nexit 3\n")
    colcon.chmod(0o755)
    env["PATH"] = str(tools) + os.pathsep + env["PATH"]
    result = run_launcher(startup_environment, extra=("--build-messages",))
    assert result.returncode != 0 and "消息包编译失败" in result.stderr
    assert "START_RECORD=" not in result.stdout


@pytest.mark.parametrize("shell", ["bash", "zsh"])
def test_mismatched_ros_distro_stops_before_build_or_server(startup_environment, shell):
    ros, workspace, _, _, env = startup_environment
    (ros / f"setup.{shell}").write_text("export ROS_DISTRO=jazzy\n")
    tools = workspace / "tools"
    tools.mkdir()
    colcon = tools / "colcon"
    colcon.write_text("#!/bin/sh\ntouch build-invoked\nexit 0\n")
    colcon.chmod(0o755)
    env["PATH"] = str(tools) + os.pathsep + env["PATH"]
    result = run_launcher(startup_environment, shell, extra=("--build-messages",))
    assert result.returncode != 0
    assert "humble" in result.stderr and "jazzy" in result.stderr
    assert not (workspace / "build-invoked").exists()
    assert "START_RECORD=" not in result.stdout


def test_overlay_switching_back_to_jazzy_is_rejected(startup_environment):
    (startup_environment[1] / "install/setup.bash").write_text("export ROS_DISTRO=jazzy\n")
    result = run_launcher(startup_environment)
    assert result.returncode != 0
    assert "humble" in result.stderr and "jazzy" in result.stderr
    assert "START_RECORD=" not in result.stdout


def test_matching_isolated_overlay_is_preferred_over_legacy_jazzy_install(startup_environment):
    workspace = startup_environment[1]
    (workspace / "install/setup.bash").write_text("export ROS_DISTRO=jazzy\n")
    isolated = workspace / "install-mcp-humble"
    isolated.mkdir()
    (isolated / "setup.bash").write_text(
        "export ROS_DISTRO=humble\nexport EE_MCP_WORKSPACE_SETUP=humble-isolated\n"
    )
    data = record(run_launcher(startup_environment))
    assert data["env"]["EE_MCP_WORKSPACE_SETUP"] == "humble-isolated"


@pytest.mark.parametrize("shell", ["bash", "zsh"])
@pytest.mark.parametrize("distro,exit_code", [("humble", 0), ("jazzy", 1)])
def test_environment_helper_requires_robot_matching_ros(
    startup_environment, shell, distro, exit_code
):
    if shutil.which(shell) is None:
        pytest.skip(f"{shell} unavailable")
    ros, _, _, _, env = startup_environment
    (ros / f"setup.{shell}").write_text(f"export ROS_DISTRO={distro}\n")
    env = {**env, "EE_CONTROL_MCP_ROS_DIR": str(ros)}
    script = ROOT / "scripts/setup_ros_env.sh"
    result = subprocess.run(
        [shell, "-c", f"source {shlex.quote(str(script))}\nexit $?"],
        env=env,
        text=True,
        capture_output=True,
        timeout=10,
    )
    assert result.returncode == exit_code, result.stdout + result.stderr
    if exit_code == 0:
        assert "humble" in result.stdout and "ROS_DOMAIN_ID=60" in result.stdout
    else:
        assert "humble" in result.stderr and "jazzy" in result.stderr
