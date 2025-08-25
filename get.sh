#!/bin/bash

# Cek apakah username adalah 'peter'
if [ "$(whoami)" == "peter" ]; then
    # Menghapus semua perubahan lokal dan mengembalikan ke commit terakhir
    git reset --hard

    # Melakukan pull untuk mendapatkan update terbaru dari repositori remote
    git pull
else
    echo "you are not 'peter', can't don't force pull"
fi
