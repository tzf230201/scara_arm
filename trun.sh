./get.sh
if [ "$(whoami)" == "peter" ]; then
    python3 stepper_uirobot/c_ver/tester.py
else
    echo "you are not 'peter', can't run"
fi
