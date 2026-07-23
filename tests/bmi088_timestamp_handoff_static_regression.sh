#!/usr/bin/env bash
set -euo pipefail

MODULE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

python3 - "$MODULE_DIR/BMI088.hpp" <<'PY'
from pathlib import Path
import re
import sys


class ContractError(RuntimeError):
    pass


def require(condition: bool, description: str) -> None:
    if not condition:
        raise ContractError(f"missing: {description}")


def section(source: str, start: str, end: str) -> str:
    start_index = source.find(start)
    end_index = source.find(end, start_index + len(start))
    require(start_index >= 0 and end_index >= 0, f"code section {start}")
    return source[start_index:end_index]


def check(source: str) -> None:
    callback = section(source, "auto gyro_int_cb", "int_gyro_->SetConfig")
    monitor = section(source, "void OnMonitor", "static void ThreadFunc")
    thread = section(source, "static void ThreadFunc", "void ControlTemperature")

    require(
        re.search(
            r"LibXR::MPMCQueue\s*<\s*LibXR::MicrosecondTimestamp\s*>\s*"
            r"sample_timestamps_\s*\{\s*4\s*\}\s*;",
            source,
        ),
        "ISR timestamp queue",
    )
    require(
        re.search(
            r"std::atomic\s*<\s*uint32_t\s*>\s*gyro_interval_us_\s*\{\s*0\s*\}\s*;",
            source,
        ),
        "32-bit gyro interval snapshot",
    )
    require(
        re.search(
            r"std::atomic\s*<\s*uint32_t\s*>\s*timestamp_drop_count_\s*\{\s*0\s*\}\s*;",
            source,
        ),
        "lock-free timestamp drop counter",
    )
    require("sample_timestamp_" not in source, "removal of shared sample timestamp")

    require("Timebase::GetMicroseconds()" in callback, "ISR timestamp capture")
    require(
        re.search(
            r"auto\s+push_result\s*=\s*bmi088->sample_timestamps_\.Push\(TIMESTAMP\)\s*;",
            callback,
        ),
        "initial ISR timestamp push result",
    )
    require(
        re.search(
            r"if\s*\(push_result\s*==\s*LibXR::ErrorCode::FULL\)\s*\{.*?"
            r"const auto POP_RESULT\s*=\s*bmi088->sample_timestamps_\.Pop\(\)\s*;.*?"
            r"if\s*\(POP_RESULT\s*==\s*LibXR::ErrorCode::OK\)\s*\{.*?"
            r"push_result\s*=\s*bmi088->sample_timestamps_\.Push\(TIMESTAMP\)\s*;",
            callback,
            re.DOTALL,
        ),
        "single drop-oldest and checked retry",
    )
    require(
        callback.count("sample_timestamps_.Push(TIMESTAMP)") == 2
        and callback.count("sample_timestamps_.Pop()") == 1
        and not re.search(r"\b(?:while|for)\s*\(", callback),
        "strictly bounded ISR queue operations",
    )
    require(
        re.search(
            r"if\s*\(push_result\s*==\s*LibXR::ErrorCode::OK\)\s*\{\s*"
            r"bmi088->new_data_\.PostFromCallback\(in_isr\);\s*\}\s*else\s*\{\s*"
            r"bmi088->timestamp_drop_count_\.fetch_add\(1,\s*std::memory_order_relaxed\);",
            callback,
        ),
        "post only on enqueue success and count drops",
    )
    require(callback.count("PostFromCallback(in_isr)") == 1, "single ISR semaphore post")
    for forbidden in ("dt_gyro_", "last_gyro_int_time_", "gyro_interval_us_"):
        require(forbidden not in callback, f"ISR does not access {forbidden}")

    require(
        re.search(
            r"MicrosecondTimestamp\s+sample_timestamp\s*;.*?"
            r"sample_timestamps_\.Pop\(sample_timestamp\).*?"
            r"while\s*\(\s*bmi088->sample_timestamps_\.Pop\(newest_timestamp\)\s*"
            r"==\s*LibXR::ErrorCode::OK\s*\)",
            thread,
            re.DOTALL,
        ),
        "task pop and newest timestamp drain",
    )
    require(
        re.search(
            r"while\s*\(\s*bmi088->sample_timestamps_\.Pop\(newest_timestamp\)\s*"
            r"==\s*LibXR::ErrorCode::OK\s*\)\s*\{.*?"
            r"const auto TOKEN_RESULT\s*=\s*bmi088->new_data_\.Wait\(0\)\s*;.*?"
            r"if\s*\(TOKEN_RESULT\s*!=\s*LibXR::ErrorCode::OK\)\s*\{\s*break\s*;",
            thread,
            re.DOTALL,
        ),
        "non-blocking drain token consumption with race handling",
    )
    require(
        "bool has_last_gyro_int_time_ = false;" in source
        and "if (bmi088->has_last_gyro_int_time_)" in thread
        and "bmi088->has_last_gyro_int_time_ = true;" in thread,
        "first timestamp interval guard",
    )
    require(
        "sample_timestamp - bmi088->last_gyro_int_time_" in thread,
        "task-owned gyro interval calculation",
    )
    require(
        "bmi088->last_gyro_int_time_ = sample_timestamp;" in thread,
        "task-owned previous timestamp update",
    )
    require(
        re.search(
            r"gyro_interval_us_\.store\(\s*static_cast<uint32_t>\("
            r"INTERVAL\.ToMicrosecond\(\)\),\s*std::memory_order_relaxed\s*\)",
            thread,
        ),
        "task interval snapshot publication",
    )
    require(
        thread.count("bmi088->topic_accl_.Publish(bmi088->accl_data_, sample_timestamp);") == 1
        and thread.count("bmi088->topic_gyro_.Publish(bmi088->gyro_data_, sample_timestamp);") == 1,
        "newest timestamp topic publication",
    )

    require(
        len(re.findall(r"gyro_interval_us_\.load\(std::memory_order_relaxed\)", monitor)) == 1,
        "single relaxed interval snapshot load",
    )
    require(
        "static_assert(std::atomic<uint32_t>::is_always_lock_free" in source,
        "lock-free interval snapshot guarantee",
    )
    require("dt_gyro_" not in monitor, "monitor isolation from task-owned duration")
    require(
        re.search(r"const float GYRO_DT\s*=\s*static_cast<float>\(GYRO_INTERVAL_US\)", monitor),
        "monitor local interval conversion",
    )


source = Path(sys.argv[1]).read_text(encoding="utf-8")

try:
    check(source)
except ContractError as error:
    print(error, file=sys.stderr)
    raise SystemExit(1)


def bounded_handoff(first_push: str, pop_result: str = "EMPTY", retry_push: str = "FULL"):
    pushes = 1
    pops = 0
    final_push = first_push
    if first_push == "FULL":
        pops = 1
        if pop_result == "OK":
            pushes += 1
            final_push = retry_push
    posted = int(final_push == "OK")
    return pushes, pops, posted, 1 - posted


state_cases = (
    (("OK",), (1, 0, 1, 0)),
    (("FULL", "EMPTY"), (1, 1, 0, 1)),
    (("FULL", "OK", "FULL"), (2, 1, 0, 1)),
    (("FULL", "OK", "OK"), (2, 1, 1, 0)),
)
for inputs, expected in state_cases:
    if bounded_handoff(*inputs) != expected:
        print(f"bounded handoff state failure: {inputs}", file=sys.stderr)
        raise SystemExit(1)

mutations = (
    (
        "bounded ISR handoff",
        "if (push_result == LibXR::ErrorCode::FULL)",
        "while (push_result == LibXR::ErrorCode::FULL)",
    ),
    (
        "drop-oldest result gate",
        "if (POP_RESULT == LibXR::ErrorCode::OK)",
        "if (POP_RESULT != LibXR::ErrorCode::OK)",
    ),
    (
        "checked retry result",
        "              push_result = bmi088->sample_timestamps_.Push(TIMESTAMP);",
        "              bmi088->sample_timestamps_.Push(TIMESTAMP);",
    ),
    (
        "enqueue success post gate",
        "if (push_result == LibXR::ErrorCode::OK)",
        "if (push_result != LibXR::ErrorCode::OK)",
    ),
    (
        "timestamp drop accounting",
        "timestamp_drop_count_.fetch_add(1,",
        "timestamp_drop_count_.fetch_add(0,",
    ),
    (
        "ISR ownership violation",
        "bmi088->new_data_.PostFromCallback(in_isr);",
        "bmi088->dt_gyro_ = LibXR::MicrosecondTimestamp::Duration(0);\n"
        "          bmi088->new_data_.PostFromCallback(in_isr);",
    ),
    (
        "newest timestamp drain",
        "bmi088->sample_timestamps_.Pop(newest_timestamp) ==",
        "bmi088->sample_timestamps_.Pop(newest_timestamp) !=",
    ),
    (
        "drained semaphore token",
        "bmi088->new_data_.Wait(0)",
        "bmi088->new_data_.Wait(1)",
    ),
    (
        "monitor snapshot ownership",
        "const auto GYRO_INTERVAL_US =",
        "const auto GYRO_INTERVAL_US = bmi088->dt_gyro_.ToMicrosecond(); //",
    ),
)

for description, old, new in mutations:
    if old not in source:
        print(f"mutation fixture mismatch: {description}", file=sys.stderr)
        raise SystemExit(1)
    try:
        check(source.replace(old, new, 1))
    except ContractError:
        continue
    print(f"mutation survived: {description}", file=sys.stderr)
    raise SystemExit(1)

print("PASS: BMI088 timestamp ownership regression")
PY
