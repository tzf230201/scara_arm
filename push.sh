# Cek apakah username adalah bukan 'peter'
if [ "$(whoami)" != "peter" ]; then
    git status
    git add .
    git status
    git commit -m "ready for first demo"
    git push origin main --force
else
    echo "username 'peter', can't run this script."
fi
