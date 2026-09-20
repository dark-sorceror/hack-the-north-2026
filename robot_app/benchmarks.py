"""Run the five Week 1 decomposition samples and report latency."""
import argparse
import json
import time

from .task_decomposer import VLATaskDecomposer


SAMPLE_TASKS = (
    "Pick up the water bottle from the table.",
    "Place the red block in the tray.",
    "Deliver the coffee mug to the person at the start position.",
    "Pick up the blue cube and place it beside the yellow cube.",
    "Deliver the small package from the shelf to the loading area.",
)


def run(decomposer, tasks=SAMPLE_TASKS):
    results = []
    for command in tasks:
        started = time.perf_counter()
        plan = decomposer.decompose(command)
        results.append({"command": command, "latency_s": round(time.perf_counter() - started, 4),
                        "subgoals": len(plan["subgoals"])})
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    results = run(VLATaskDecomposer())
    print(json.dumps(results, indent=2) if args.json else "\n".join(
        f"{item['latency_s']:.3f}s  {item['command']}" for item in results))


if __name__ == "__main__":
    main()