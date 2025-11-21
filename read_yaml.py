#!/usr/bin/env python3
"""
Reads ILLIXR-style YAML and generates a C++ header with config constants.
Supports comma-separated or list-style plugin lists.
"""
import sys, yaml, textwrap

if len(sys.argv) != 3:
    print("Usage: read_yaml.py <yaml_file> <output_header>")
    sys.exit(1)

yaml_path, header_path = sys.argv[1], sys.argv[2]
data = yaml.safe_load(open(yaml_path))

def as_bool(val):
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.lower() in ["true", "1", "yes", "on"]
    return False

duration = data.get("duration", 0)

# Two distinct directories
data_path = data.get("data", "")
demo_data_path = data.get("demo_data", "")

# Plugins and visualizers
plugins_field = data.get("plugins", [])
if(plugins_field is None):
    plugins = []
elif isinstance(plugins_field, str):
    plugins = [p.strip() for p in plugins_field.split(",") if p.strip()]
else:
    plugins = list(plugins_field)

visualizers = data.get("visualizers", "")
if isinstance(visualizers, str) and visualizers:
    plugins.append(visualizers)

# Boolean flags
enable_offload   = as_bool(data.get("enable_offload", False))
enable_alignment = as_bool(data.get("enable_alignment", False))
verbose_errors   = as_bool(data.get("enable_verbose_errors", False))
enable_pre_sleep = as_bool(data.get("enable_pre_sleep", False))

# Emit header
header = textwrap.dedent(f"""\
    // Auto-generated from {yaml_path}
    #pragma once
    constexpr int RUN_DURATION = {duration};
    constexpr char DATA_PATH[] = "{data_path}";
    constexpr char DEMO_DATA_PATH[] = "{demo_data_path}";
    constexpr bool ENABLE_OFFLOAD = {"true" if enable_offload else "false"};
    constexpr bool ENABLE_ALIGNMENT = {"true" if enable_alignment else "false"};
    constexpr bool ENABLE_VERBOSE_ERRORS = {"true" if verbose_errors else "false"};
    constexpr bool ENABLE_PRE_SLEEP = {"true" if enable_pre_sleep else "false"};
    constexpr const char* PLUGINS[] = {{
        {", ".join(f'"{p}"' for p in plugins)}, nullptr
    }};
""")

with open(header_path, "w") as f:
    f.write(header)
print(f"[read_yaml] Wrote {header_path}")
