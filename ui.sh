./get.sh
if [ "$(whoami)" == "peter" ]; then
    python3 stepper_uirobot/e_ver/gui.py
else
    echo "you are not 'peter', can't run"
fi
