# main.py
from multiprocessing import Process, Queue
import time

def run_gui(queue):
    import tkinter as tk

    def kirim_data():
        nama = entry.get()
        queue.put(nama)  # kirim ke proses utama

    root = tk.Tk()
    root.title("GUI di Proses Terpisah")
    root.geometry("300x200")

    tk.Label(root, text="Masukkan nama:").pack(pady=5)
    entry = tk.Entry(root)
    entry.pack(pady=5)

    tk.Button(root, text="Kirim", command=kirim_data).pack(pady=5)

    root.mainloop()

if __name__ == "__main__":
    queue = Queue()

    gui_process = Process(target=run_gui, args=(queue,))
    gui_process.start()

    try:
        while True:
            if not queue.empty():
                data = queue.get()
                print(f"[MAIN PROCESS] Dapat nama: {data}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Menutup program...")
    finally:
        gui_process.terminate()
        gui_process.join()
