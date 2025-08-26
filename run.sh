./get.sh
if [ "$(whoami)" == "peter" ]; then
    python3 main.py
else
    echo "you are not 'peter', can't run"
fi
