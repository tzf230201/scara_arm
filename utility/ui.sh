./get.sh
if [ "$(whoami)" == "peter" ]; then
    python3 src/f_ver/gui.py
else
    echo "you are not 'peter', can't run"
fi
