#!/bin/zsh
# Usage: keylines.sh <log>  — the lines an arm is judged by, timestamps stripped.
grep -E '==> coupler insertion|CHECK|\[Coupler Insertion\] chain [0-9]+:|feedline routing round|failed:|final routing:|is open in the room of|room rules —|stands down' "$1" \
  | sed -E 's/^\[final\] +[0-9.]+s  //; s/, [0-9.]+s$//'
