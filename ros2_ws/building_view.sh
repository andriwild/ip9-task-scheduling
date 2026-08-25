#!/bin/zsh

#
# BEGIN AI-assisted: generated with Claude Code (Anthropic),
# reviewed and adapted by the author.
#
CONFIG_DIR=${0:A:h}/config
PORT=${1:-8777}

python3 -m http.server $PORT --bind 127.0.0.1 --directory $CONFIG_DIR >/dev/null 2>&1 &
SERVER=$!
trap "kill $SERVER 2>/dev/null" EXIT INT TERM

until curl -s -o /dev/null http://127.0.0.1:$PORT/building_view.html; do
    sleep 0.2
done

echo "building_view: http://127.0.0.1:$PORT/building_view.html (Ctrl-C zum Beenden)"
xdg-open "http://127.0.0.1:$PORT/building_view.html" >/dev/null 2>&1
wait $SERVER
