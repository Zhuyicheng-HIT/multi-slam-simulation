# Script Ownership

This directory will contain thin, fail-fast entrypoints for build, rosbag2 record/replay, baseline flight, offline evaluation, and milestone regression matrices.

Scripts must derive the repository root from their own location, accept external dependency paths through documented environment variables, write generated data under ignored output directories, and return nonzero when an acceptance check fails.
