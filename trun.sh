./get.sh
if [ "$(whoami)" == "peter" ]; then
    python3 stepper_uirobot/b_ver/tester.py
else
    echo "you are not 'peter', can't run"
fi
