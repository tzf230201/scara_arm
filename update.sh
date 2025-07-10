# Cek apakah username adalah bukan 'peter'
if [ "$(whoami)" != "peter" ]; then
    git status
    git add .
    git status
    git commit -m "new pvt testing" *
    git push origin main
else
    echo "pengguna 'peter', tidak dapat menjalankan script ini."
fi