./get.sh
if [ "$(whoami)" == "peter" ]; then
    python3 tester.py
else
    echo "you are not 'peter', can't run"
fi
