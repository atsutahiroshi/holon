#!/usr/bin/env bash

if [[ -z "$1" ]]; then
  echo "usage:"
  echo "  ${0} <executalbe file path in example>"
  echo "example:"
  echo "  ${0} holon/example/humanoid/com_regulation_example"
  exit 1
fi

# Assuming example to be executed is `holon/example/humanoid/com_regulation_example`,
# data file should be `holon/example/humanoid/com_regulation_example.dat` and
# script file should be `holon/example/humanoid/tools/com_regulation_example_plot.py`
EXE_CMD="$1"
EXE_FILENAME=$(basename "$EXE_CMD")
EXE_DIR=$(dirname "$EXE_CMD")
DAT_PATH="$EXE_DIR/${EXE_FILENAME}.dat"
PLOT_SCRIPT="$EXE_DIR/tools/${EXE_FILENAME}_plot.py"

if [[ -f CMakeCache.txt ]]; then
  cmake --build . -- -j4
fi

if [[ -f $EXE_CMD ]]; then
  $EXE_CMD > $DAT_PATH
else
  echo "$EXE_CMD does not exist."
fi

cmd_is_available () {
  return $(command -v $1 >/dev/null)
}

pipenv_is_available () {
  cmd_is_available pipenv && [[ -f Pipfile ]]
  return $?
}

if [[ -f $PLOT_SCRIPT ]]; then
  if pipenv_is_available; then
    pipenv run python $PLOT_SCRIPT $DAT_PATH
  else
    python $PLOT_SCRIPT $DAT_PATH
  fi
else
  echo "$PLOT_SCRIPT does not exist."
fi
