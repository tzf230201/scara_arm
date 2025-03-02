from vpython import *

# Membuat bola
bola = sphere(pos=vector(-5, 0, 0), radius=0.5, color=color.red)

# Membuat lantai
lantai = box(pos=vector(0, -1, 0), size=vector(10, 0.2, 10), color=color.green)

# Membuat panah (untuk menunjukkan arah gerak bola)
panah = arrow(pos=bola.pos, axis=vector(2, 0, 0), color=color.yellow)

# Kecepatan bola
kecepatan = vector(0.1, 0, 0)

# Loop animasi
while True:
    rate(50)  # Menjalankan loop 50 kali per detik
    
    # Perbarui posisi bola
    bola.pos += kecepatan
    
    # Perbarui posisi panah
    panah.pos = bola.pos
    
    # Pantulan jika bola mencapai tepi lantai
    if bola.pos.x > 5 or bola.pos.x < -5:
        kecepatan.x = -kecepatan.x
